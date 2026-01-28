import math
import os
import sys
import time
from dataclasses import dataclass, field
from typing import List, Optional

import pyqtgraph as pg
from pyqtgraph.exporters import PDFExporter
from PyQt5 import QtCore, QtWidgets
import serial
import serial.tools.list_ports


@dataclass
class Sample:
    timestamp_ms: int
    flow_slm: float
    volume_l: float
    volume_btps_l: float
    temp_c: float
    humidity: float


@dataclass
class EnvironmentSample:
    timestamp_ms: int
    temp_c: float
    humidity: float


@dataclass
class Maneuver:
    start_time_ms: Optional[int] = None
    samples: List[Sample] = field(default_factory=list)

    def reset(self) -> None:
        self.start_time_ms = None
        self.samples.clear()

    def add_sample(self, sample: Sample) -> None:
        if self.start_time_ms is None:
            self.start_time_ms = sample.timestamp_ms
        self.samples.append(sample)

    def compute_fev1(self) -> Optional[float]:
        if self.start_time_ms is None:
            return None
        target_time = self.start_time_ms + 1000
        for sample in self.samples:
            if sample.timestamp_ms >= target_time:
                return sample.volume_btps_l
        return None

    def compute_fvc(self) -> Optional[float]:
        if not self.samples:
            return None
        return max(sample.volume_btps_l for sample in self.samples)

    def compute_pef(self) -> Optional[float]:
        if not self.samples:
            return None
        return max(sample.flow_slm for sample in self.samples)


class SpirometryApp(QtWidgets.QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Spirometria Open-Source")
        self.resize(1200, 800)

        self.flow_port: Optional[serial.Serial] = None
        self.env_port: Optional[serial.Serial] = None
        self.flow_buffer = ""
        self.env_buffer = ""
        self.samples: List[Sample] = []
        self.maneuver = Maneuver()
        self.last_flow_timestamp_ms: Optional[int] = None
        self.last_flow_slm = 0.0
        self.current_temp_c = 25.0
        self.current_humidity = 50.0
        self.last_fev1: Optional[float] = None
        self.last_fvc: Optional[float] = None
        self.last_tiffeneau: Optional[float] = None
        self.demo_enabled = False
        self.demo_start_time = time.monotonic()

        self._build_ui()
        self._configure_plots()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self._read_serial)
        self.demo_timer = QtCore.QTimer()
        self.demo_timer.timeout.connect(self._emit_demo_sample)

    def _build_ui(self) -> None:
        central = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(central)

        control_layout = QtWidgets.QHBoxLayout()
        self.refresh_button = QtWidgets.QPushButton("Aggiorna porte")
        self.flow_port_combo = QtWidgets.QComboBox()
        self.env_port_combo = QtWidgets.QComboBox()
        self.flow_baud_combo = QtWidgets.QComboBox()
        self.flow_baud_combo.addItems(["115200", "230400", "460800"])
        self.env_baud_combo = QtWidgets.QComboBox()
        self.env_baud_combo.addItems(["115200", "9600"])
        self.connect_flow_button = QtWidgets.QPushButton("Connetti Flusso")
        self.disconnect_flow_button = QtWidgets.QPushButton("Disconnetti Flusso")
        self.connect_env_button = QtWidgets.QPushButton("Connetti Ambiente")
        self.disconnect_env_button = QtWidgets.QPushButton("Disconnetti Ambiente")
        self.export_button = QtWidgets.QPushButton("Esporta PDF")
        self.demo_checkbox = QtWidgets.QCheckBox("Modalità demo")

        control_layout.addWidget(QtWidgets.QLabel("Porta Flusso"))
        control_layout.addWidget(self.flow_port_combo)
        control_layout.addWidget(self.refresh_button)
        control_layout.addWidget(QtWidgets.QLabel("Baud Flusso"))
        control_layout.addWidget(self.flow_baud_combo)
        control_layout.addWidget(self.connect_flow_button)
        control_layout.addWidget(self.disconnect_flow_button)
        control_layout.addWidget(QtWidgets.QLabel("Porta Ambiente"))
        control_layout.addWidget(self.env_port_combo)
        control_layout.addWidget(QtWidgets.QLabel("Baud Ambiente"))
        control_layout.addWidget(self.env_baud_combo)
        control_layout.addWidget(self.connect_env_button)
        control_layout.addWidget(self.disconnect_env_button)
        control_layout.addWidget(self.demo_checkbox)
        control_layout.addStretch()
        control_layout.addWidget(self.export_button)

        layout.addLayout(control_layout)

        plots_layout = QtWidgets.QGridLayout()
        self.flow_plot = pg.PlotWidget(title="Flusso - Tempo")
        self.volume_plot = pg.PlotWidget(title="Volume - Tempo (BTPS)")
        self.flow_volume_plot = pg.PlotWidget(title="Flusso - Volume (BTPS)")

        plots_layout.addWidget(self.flow_plot, 0, 0)
        plots_layout.addWidget(self.volume_plot, 0, 1)
        plots_layout.addWidget(self.flow_volume_plot, 1, 0, 1, 2)

        layout.addLayout(plots_layout)

        self.metrics_label = QtWidgets.QLabel(
            "FEV1: -- L | FVC (CVF): -- L | Indice di Tiffeneau: -- | PEF: -- SLM"
        )
        self.hardware_label = QtWidgets.QLabel(
            "Connessione: il flusso arriva via SEK-SFM3XXX-AW/D (porta seriale PC). "
            "ESP32 invia solo temperatura/umidità DHT11."
        )
        layout.addWidget(self.metrics_label)
        layout.addWidget(self.hardware_label)

        self.setCentralWidget(central)

        self.refresh_button.clicked.connect(self._refresh_ports)
        self.connect_flow_button.clicked.connect(self._connect_flow_serial)
        self.disconnect_flow_button.clicked.connect(self._disconnect_flow_serial)
        self.connect_env_button.clicked.connect(self._connect_env_serial)
        self.disconnect_env_button.clicked.connect(self._disconnect_env_serial)
        self.export_button.clicked.connect(self._export_pdf)
        self.demo_checkbox.toggled.connect(self._toggle_demo_mode)

        self._refresh_ports()

    def _configure_plots(self) -> None:
        self.flow_curve = self.flow_plot.plot(pen=pg.mkPen("#2E86AB", width=2))
        self.volume_curve = self.volume_plot.plot(pen=pg.mkPen("#28B463", width=2))
        self.flow_volume_curve = self.flow_volume_plot.plot(pen=pg.mkPen("#AF7AC5", width=2))

        self.flow_plot.setLabel("left", "Flusso", units="SLM")
        self.flow_plot.setLabel("bottom", "Tempo", units="s")

        self.volume_plot.setLabel("left", "Volume", units="L")
        self.volume_plot.setLabel("bottom", "Tempo", units="s")

        self.flow_volume_plot.setLabel("left", "Flusso", units="SLM")
        self.flow_volume_plot.setLabel("bottom", "Volume", units="L")

    def _refresh_ports(self) -> None:
        self.flow_port_combo.clear()
        self.env_port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.flow_port_combo.addItem(port.device)
            self.env_port_combo.addItem(port.device)

    def _connect_flow_serial(self) -> None:
        if self.flow_port and self.flow_port.is_open:
            return
        port_name = self.flow_port_combo.currentText()
        if not port_name:
            QtWidgets.QMessageBox.warning(self, "Porta non selezionata", "Seleziona una porta seriale.")
            return
        baud_rate = int(self.flow_baud_combo.currentText())
        try:
            self.flow_port = serial.Serial(port_name, baud_rate, timeout=0.1)
        except serial.SerialException as exc:
            QtWidgets.QMessageBox.critical(self, "Errore", str(exc))
            return

        self.flow_buffer = ""
        self.samples.clear()
        self.maneuver.reset()
        self.last_flow_timestamp_ms = None
        self.last_flow_slm = 0.0
        self.timer.start(20)

    def _connect_env_serial(self) -> None:
        if self.env_port and self.env_port.is_open:
            return
        port_name = self.env_port_combo.currentText()
        if not port_name:
            QtWidgets.QMessageBox.warning(self, "Porta non selezionata", "Seleziona una porta seriale.")
            return
        baud_rate = int(self.env_baud_combo.currentText())
        try:
            self.env_port = serial.Serial(port_name, baud_rate, timeout=0.1)
        except serial.SerialException as exc:
            QtWidgets.QMessageBox.critical(self, "Errore", str(exc))
            return
        self.env_buffer = ""
        self.timer.start(20)

    def _disconnect_flow_serial(self) -> None:
        if self.flow_port:
            self.flow_port.close()
            self.flow_port = None
        if not self.env_port:
            self.timer.stop()

    def _disconnect_env_serial(self) -> None:
        if self.env_port:
            self.env_port.close()
            self.env_port = None
        if not self.flow_port:
            self.timer.stop()

    def _toggle_demo_mode(self, enabled: bool) -> None:
        self.demo_enabled = enabled
        if enabled:
            self._disconnect_flow_serial()
            self._disconnect_env_serial()
            self.samples.clear()
            self.maneuver.reset()
            self.last_flow_timestamp_ms = None
            self.last_flow_slm = 0.0
            self.demo_start_time = time.monotonic()
            self.demo_timer.start(50)
        else:
            self.demo_timer.stop()

    def _parse_flow_line(self, line: str) -> Optional[Sample]:
        if not line or line.startswith("timestamp"):
            return None
        parts = line.strip().split(",")
        try:
            if len(parts) == 1:
                timestamp_ms = int(time.monotonic() * 1000)
                flow_slm = float(parts[0])
            elif len(parts) >= 2:
                timestamp_ms = int(parts[0])
                flow_slm = float(parts[1])
            else:
                return None
        except ValueError:
            return None

        volume_l, volume_btps_l = self._update_volume(flow_slm, timestamp_ms)
        return Sample(
            timestamp_ms=timestamp_ms,
            flow_slm=flow_slm,
            volume_l=volume_l,
            volume_btps_l=volume_btps_l,
            temp_c=self.current_temp_c,
            humidity=self.current_humidity,
        )

    def _parse_env_line(self, line: str) -> Optional[EnvironmentSample]:
        if not line or line.startswith("timestamp"):
            return None
        parts = line.strip().split(",")
        if len(parts) < 3:
            return None
        try:
            return EnvironmentSample(
                timestamp_ms=int(parts[0]),
                temp_c=float(parts[1]),
                humidity=float(parts[2]),
            )
        except ValueError:
            return None

    def _update_maneuver(self, sample: Sample) -> None:
        threshold = 2.0
        if sample.flow_slm > threshold:
            self.maneuver.add_sample(sample)
        elif self.maneuver.samples:
            if sample.flow_slm < 0.5:
                self._finalize_maneuver()

    def _finalize_maneuver(self) -> None:
        fev1 = self.maneuver.compute_fev1()
        fvc = self.maneuver.compute_fvc()
        pef = self.maneuver.compute_pef()
        tiffeneau = (fev1 / fvc) if (fev1 is not None and fvc) else None

        fev1_text = f"{fev1:.2f}" if fev1 is not None else "--"
        fvc_text = f"{fvc:.2f}" if fvc is not None else "--"
        pef_text = f"{pef:.2f}" if pef is not None else "--"
        tiff_text = f"{tiffeneau:.2f}" if tiffeneau is not None else "--"

        self.metrics_label.setText(
            "FEV1: "
            f"{fev1_text} L | FVC (CVF): {fvc_text} L | "
            f"Indice di Tiffeneau: {tiff_text} | PEF: {pef_text} SLM"
        )
        self.last_fev1 = fev1
        self.last_fvc = fvc
        self.last_tiffeneau = tiffeneau
        self.maneuver.reset()

    def _read_serial(self) -> None:
        if self.demo_enabled:
            return
        if self.flow_port:
            try:
                raw = self.flow_port.read(1024).decode("utf-8", errors="ignore")
            except serial.SerialException:
                raw = ""
            if raw:
                self.flow_buffer += raw
                lines = self.flow_buffer.split("\n")
                self.flow_buffer = lines[-1]

                for line in lines[:-1]:
                    sample = self._parse_flow_line(line)
                    if sample:
                        self.samples.append(sample)
                        self._update_maneuver(sample)

        if self.env_port:
            try:
                raw_env = self.env_port.read(1024).decode("utf-8", errors="ignore")
            except serial.SerialException:
                raw_env = ""
            if raw_env:
                self.env_buffer += raw_env
                env_lines = self.env_buffer.split("\n")
                self.env_buffer = env_lines[-1]
                for line in env_lines[:-1]:
                    env_sample = self._parse_env_line(line)
                    if env_sample:
                        self.current_temp_c = env_sample.temp_c
                        self.current_humidity = env_sample.humidity

        self._update_plots()

    def _emit_demo_sample(self) -> None:
        elapsed_s = time.monotonic() - self.demo_start_time
        timestamp_ms = int(elapsed_s * 1000)
        flow_slm = self._demo_flow_profile(elapsed_s)
        self.current_temp_c = 23.0 + 2.0 * math.sin(elapsed_s / 15.0)
        self.current_humidity = 50.0 + 5.0 * math.sin(elapsed_s / 20.0)
        volume_l, volume_btps_l = self._update_volume(flow_slm, timestamp_ms)
        sample = Sample(
            timestamp_ms=timestamp_ms,
            flow_slm=flow_slm,
            volume_l=volume_l,
            volume_btps_l=volume_btps_l,
            temp_c=self.current_temp_c,
            humidity=self.current_humidity,
        )
        self.samples.append(sample)
        self._update_maneuver(sample)
        self._update_plots()

    @staticmethod
    def _demo_flow_profile(elapsed_s: float) -> float:
        cycle = elapsed_s % 8.0
        if cycle < 1.0:
            return 1.0 + 3.0 * cycle
        if cycle < 2.0:
            return 4.0 - 2.0 * (cycle - 1.0)
        if cycle < 3.0:
            return -1.0 - 3.0 * (cycle - 2.0)
        if cycle < 4.0:
            return -4.0 + 3.0 * (cycle - 3.0)
        return 0.3 * math.sin(2 * math.pi * (cycle - 4.0))

    def _update_volume(self, flow_slm: float, timestamp_ms: int) -> tuple[float, float]:
        if self.last_flow_timestamp_ms is None:
            self.last_flow_timestamp_ms = timestamp_ms
            self.last_flow_slm = flow_slm
            return 0.0, 0.0

        dt_s = (timestamp_ms - self.last_flow_timestamp_ms) / 1000.0
        flow_ls = flow_slm / 60.0
        last_flow_ls = self.last_flow_slm / 60.0
        volume_l = (self.samples[-1].volume_l if self.samples else 0.0) + 0.5 * (flow_ls + last_flow_ls) * dt_s

        btp_factor = self._compute_btp_factor(self.current_temp_c, self.current_humidity)
        volume_btps_l = volume_l * btp_factor

        self.last_flow_timestamp_ms = timestamp_ms
        self.last_flow_slm = flow_slm
        return volume_l, volume_btps_l

    @staticmethod
    def _compute_btp_factor(temperature_c: float, humidity: float) -> float:
        tbpts_c = 37.0
        pbpts_kpa = 101.325
        t_atps_k = temperature_c + 273.15
        t_btps_k = tbpts_c + 273.15
        ph2o_atps = SpirometryApp._saturation_vapor_pressure_kpa(temperature_c) * (humidity / 100.0)
        ph2o_btps = SpirometryApp._saturation_vapor_pressure_kpa(tbpts_c)
        numerator = t_btps_k * (pbpts_kpa - ph2o_atps)
        denominator = t_atps_k * (pbpts_kpa - ph2o_btps)
        return numerator / denominator

    @staticmethod
    def _saturation_vapor_pressure_kpa(temperature_c: float) -> float:
        return 0.61078 * math.exp((17.2694 * temperature_c) / (temperature_c + 237.3))

    def _update_plots(self) -> None:
        if not self.samples:
            return
        times = [(s.timestamp_ms - self.samples[0].timestamp_ms) / 1000.0 for s in self.samples]
        flows = [s.flow_slm for s in self.samples]
        volumes = [s.volume_btps_l for s in self.samples]

        self.flow_curve.setData(times, flows)
        self.volume_curve.setData(times, volumes)
        self.flow_volume_curve.setData(volumes, flows)

    def _export_pdf(self) -> None:
        if not self.samples:
            QtWidgets.QMessageBox.warning(self, "Nessun dato", "Acquisisci i dati prima di esportare.")
            return
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Salva PDF", "spirometria.pdf", "PDF (*.pdf)")
        if not path:
            return

        times_s = [s.timestamp_ms / 1000.0 for s in self.samples]
        flows = [s.flow_slm for s in self.samples]
        volumes = [s.volume_btps_l for s in self.samples]

        fev1_text = f"{self.last_fev1:.2f} L" if self.last_fev1 is not None else "--"
        fvc_text = f"{self.last_fvc:.2f} L" if self.last_fvc is not None else "--"
        tiff_text = f"{self.last_tiffeneau:.2f}" if self.last_tiffeneau is not None else "--"

        pdf_widget = pg.GraphicsLayoutWidget(show=False)
        pdf_widget.resize(1400, 1000)

        metrics_plot = pdf_widget.addPlot()
        metrics_plot.hideAxis("bottom")
        metrics_plot.hideAxis("left")
        metrics_text = (
            f"FEV1: {fev1_text}    "
            f"FVC (CVF): {fvc_text}    "
            f"Indice di Tiffeneau: {tiff_text}"
        )
        text_item = pg.TextItem(metrics_text, color="k")
        metrics_plot.addItem(text_item)
        text_item.setPos(0, 0)

        pdf_widget.nextRow()

        flow_plot = pdf_widget.addPlot(title="Flusso - Tempo")
        flow_plot.plot(times_s, flows, pen="b")
        flow_plot.setLabel("bottom", "Tempo", units="s")
        flow_plot.setLabel("left", "Flusso", units="SLM")

        pdf_widget.nextRow()

        volume_plot = pdf_widget.addPlot(title="Volume - Tempo (BTPS)")
        volume_plot.plot(times_s, volumes, pen="g")
        volume_plot.setLabel("bottom", "Tempo", units="s")
        volume_plot.setLabel("left", "Volume", units="L")

        pdf_widget.nextRow()

        flow_volume_plot = pdf_widget.addPlot(title="Flusso - Volume (BTPS)")
        flow_volume_plot.plot(volumes, flows, pen="m")
        flow_volume_plot.setLabel("bottom", "Volume", units="L")
        flow_volume_plot.setLabel("left", "Flusso", units="SLM")

        exporter = PDFExporter(pdf_widget.scene())
        exporter.export(path)


def main() -> None:
    app = QtWidgets.QApplication(sys.argv)
    pg.setConfigOptions(antialias=True)
    window = SpirometryApp()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
