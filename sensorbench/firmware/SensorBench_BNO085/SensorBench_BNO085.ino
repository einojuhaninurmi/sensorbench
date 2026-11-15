#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <math.h>
#include <stdio.h>

// GPS and BNO085 share the confirmed WROOM-32D controller; allow a BNO-only build.
#ifndef SENSORBENCH_ENABLE_GPS
#define SENSORBENCH_ENABLE_GPS 1
#endif

#if SENSORBENCH_ENABLE_GPS
#include <TinyGPSPlus.h>
HardwareSerial GPSSerial(2);
TinyGPSPlus gps;
#endif

static const char PROTOCOL[] = "sensorbench-quaternion-v1";
static const char FIRMWARE_VERSION[] = "sensorbench-esp32-0.2.0";
static constexpr uint32_t PERIOD_MS = 100;
static constexpr uint32_t REPORT_INTERVAL_US = PERIOD_MS * 1000;
static constexpr uint32_t GPS_MAX_AGE_MS = 2000;

// Preserve the working sketch's default Wire bus, pins and I2C address.
Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t sensorValue;
bool imuReady = false;
uint32_t sampleSequence = 0;

#if SENSORBENCH_ENABLE_GPS
char gpsUtc[32] = "";
bool gpsUtcAvailable = false;
uint32_t gpsUtcReceivedAtMs = 0;
#endif

// All strings passed here are controlled constants without JSON special chars.
void emitEvent(const char *eventName, const char *message) {
  Serial.print(F("{\"type\":\"event\",\"protocol\":\""));
  Serial.print(PROTOCOL);
  Serial.print(F("\",\"firmware_version\":\""));
  Serial.print(FIRMWARE_VERSION);
  Serial.print(F("\",\"event\":\""));
  Serial.print(eventName);
  Serial.print(F("\",\"period_ms\":"));
  Serial.print(PERIOD_MS);
  Serial.print(F(",\"gps_enabled\":"));
  Serial.print(SENSORBENCH_ENABLE_GPS ? F("true") : F("false"));
  if (message != nullptr) {
    Serial.print(F(",\"message\":\""));
    Serial.print(message);
    Serial.print(F("\""));
  }
  Serial.println(F("}"));
}

bool enableRotationReport() {
  if (bno08x.enableReport(SH2_ROTATION_VECTOR, REPORT_INTERVAL_US)) {
    return true;
  }
  emitEvent("error", "Could not enable rotation-vector reports");
  return false;
}

#if SENSORBENCH_ENABLE_GPS
void updateGpsUtcSnapshot() {
  // RMC commits date and time together. GGA can update time alone, so cache
  // paired updates rather than combining a new time with yesterday's date.
  if (!gps.date.isUpdated() || !gps.time.isUpdated()) {
    return;
  }
  const bool fresh = gps.date.isValid() && gps.time.isValid() &&
                     gps.date.age() <= GPS_MAX_AGE_MS &&
                     gps.time.age() <= GPS_MAX_AGE_MS;
  const uint16_t year = gps.date.year();
  const uint8_t month = gps.date.month();
  const uint8_t day = gps.date.day();
  const uint8_t hour = gps.time.hour();
  const uint8_t minute = gps.time.minute();
  const uint8_t second = gps.time.second();
  const uint16_t millisecond = gps.time.centisecond() * 10;

  if (!fresh || year < 2000 || year > 2099 || month < 1 || month > 12 ||
      day < 1 || day > 31 || hour > 23 || minute > 59 || second > 59) {
    gpsUtcAvailable = false;
    return;
  }
  const int written = snprintf(
      gpsUtc, sizeof(gpsUtc), "%04u-%02u-%02uT%02u:%02u:%02u.%03uZ",
      static_cast<unsigned>(year), static_cast<unsigned>(month),
      static_cast<unsigned>(day), static_cast<unsigned>(hour),
      static_cast<unsigned>(minute), static_cast<unsigned>(second),
      static_cast<unsigned>(millisecond));
  gpsUtcAvailable = written > 0 &&
                    static_cast<size_t>(written) < sizeof(gpsUtc);
  gpsUtcReceivedAtMs = millis();
}

void drainGps() {
  while (GPSSerial.available() > 0) {
    if (gps.encode(static_cast<char>(GPSSerial.read()))) {
      updateGpsUtcSnapshot();
    }
  }
}

bool hasFreshGpsUtc() {
  return gpsUtcAvailable && gps.date.isValid() && gps.time.isValid() &&
         gps.date.age() <= GPS_MAX_AGE_MS &&
         gps.time.age() <= GPS_MAX_AGE_MS &&
         static_cast<uint32_t>(millis() - gpsUtcReceivedAtMs) <= GPS_MAX_AGE_MS;
}
#endif

void emitSample(const sh2_SensorValue_t &value) {
  const auto &q = value.un.rotationVector;
  const double normSquared = static_cast<double>(q.real) * q.real +
                             static_cast<double>(q.i) * q.i +
                             static_cast<double>(q.j) * q.j +
                             static_cast<double>(q.k) * q.k;
  if (!isfinite(q.real) || !isfinite(q.i) || !isfinite(q.j) ||
      !isfinite(q.k) || !isfinite(normSquared) || normSquared <= 1e-12) {
    emitEvent("error", "Invalid quaternion report");
    return;
  }
  const uint32_t sampledAtMs = millis();
  bool gpsFix = false;
#if SENSORBENCH_ENABLE_GPS
  const double latitude = gps.location.lat();
  const double longitude = gps.location.lng();
  gpsFix = gps.location.isValid() && gps.location.age() <= GPS_MAX_AGE_MS &&
           isfinite(latitude) && isfinite(longitude) &&
           latitude >= -90.0 && latitude <= 90.0 &&
           longitude >= -180.0 && longitude <= 180.0;
#endif

  Serial.print(F("{\"type\":\"sample\",\"protocol\":\""));
  Serial.print(PROTOCOL);
  Serial.print(F("\",\"firmware_version\":\""));
  Serial.print(FIRMWARE_VERSION);
  Serial.print(F("\",\"seq\":"));
  Serial.print(sampleSequence++);
  Serial.print(F(",\"t_ms\":"));
  Serial.print(sampledAtMs);
  Serial.print(F(",\"period_ms\":"));
  Serial.print(PERIOD_MS);
  Serial.print(F(",\"quaternion\":{\"w\":"));
  Serial.print(q.real, 6);
  Serial.print(F(",\"x\":"));
  Serial.print(q.i, 6);
  Serial.print(F(",\"y\":"));
  Serial.print(q.j, 6);
  Serial.print(F(",\"z\":"));
  Serial.print(q.k, 6);
  Serial.print(F("},\"sensor_status\":"));
  Serial.print(static_cast<unsigned>(value.status & 0x03));
  Serial.print(F(",\"gps_enabled\":"));
  Serial.print(SENSORBENCH_ENABLE_GPS ? F("true") : F("false"));
  Serial.print(F(",\"gps_fix\":"));
  Serial.print(gpsFix ? F("true") : F("false"));
#if SENSORBENCH_ENABLE_GPS
  if (gpsFix) {
    Serial.print(F(",\"latitude_deg\":"));
    Serial.print(latitude, 6);
    Serial.print(F(",\"longitude_deg\":"));
    Serial.print(longitude, 6);
  }
  // Clock availability is independent of having a valid position fix.
  if (hasFreshGpsUtc()) {
    Serial.print(F(",\"gps_utc\":\""));
    Serial.print(gpsUtc);
    Serial.print(F("\""));
  }
#endif
  Serial.println(F("}"));
}

void setup() {
  Serial.begin(115200);
#if SENSORBENCH_ENABLE_GPS
  GPSSerial.begin(9600, SERIAL_8N1, 16, 17);
#endif
  if (!bno08x.begin_I2C()) {
    emitEvent("error", "BNO08x not found on the default I2C bus");
    return;
  }
  imuReady = enableRotationReport();
  if (imuReady) {
    emitEvent("ready", nullptr);
  }
}

void loop() {
#if SENSORBENCH_ENABLE_GPS
  drainGps();
#endif
  if (!imuReady) {
    // Stay responsive without fabricating samples after initialization fails.
    delay(1);
    return;
  }
  if (bno08x.wasReset()) {
    emitEvent("sensor_reset", nullptr);
    imuReady = enableRotationReport();
    if (!imuReady) {
      delay(1);
      return;
    }
  }
  if (bno08x.getSensorEvent(&sensorValue) &&
      sensorValue.sensorId == SH2_ROTATION_VECTOR) {
    emitSample(sensorValue);
  }
  delay(1);
}
