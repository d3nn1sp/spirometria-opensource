#include <Arduino.h>
#include <DHT.h>

// DHT11 settings
static constexpr uint8_t kDhtPin = 4;
static constexpr uint8_t kDhtType = DHT11;

// Sampling settings
static constexpr uint32_t kSampleIntervalMs = 1000;  // 1 Hz
static constexpr uint32_t kDhtIntervalMs = 1000;

DHT dht(kDhtPin, kDhtType);

static uint32_t last_sample_ms = 0;

static float ambient_temperature_c = 25.0f;
static float ambient_humidity = 50.0f;

void setup() {
  Serial.begin(115200);
  dht.begin();

  const float initial_temp = dht.readTemperature();
  const float initial_humidity = dht.readHumidity();
  if (!isnan(initial_temp) && !isnan(initial_humidity)) {
    ambient_temperature_c = initial_temp;
    ambient_humidity = initial_humidity;
  }

  last_sample_ms = millis();

  Serial.println("timestamp_ms,temp_c,humidity");
}

void loop() {
  const uint32_t now_ms = millis();
  if (now_ms - last_sample_ms < kSampleIntervalMs) {
    return;
  }

  if (now_ms % kDhtIntervalMs < kSampleIntervalMs) {
    const float new_temp = dht.readTemperature();
    const float new_humidity = dht.readHumidity();
    if (!isnan(new_temp) && !isnan(new_humidity)) {
      ambient_temperature_c = new_temp;
      ambient_humidity = new_humidity;
    }
  }

  last_sample_ms = now_ms;

  Serial.print(now_ms);
  Serial.print(",");
  Serial.print(ambient_temperature_c, 1);
  Serial.print(",");
  Serial.println(ambient_humidity, 1);
}
