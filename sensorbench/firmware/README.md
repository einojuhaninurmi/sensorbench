# BNO085 and GPS telemetry firmware

The [ESP32 sketch](SensorBench_BNO085/SensorBench_BNO085.ino) reads a BNO085 on the default Wire I²C bus and emits quaternion telemetry as JSON Lines at 115200 baud. The GPS option reads UART2 at 9600 baud on RX GPIO16 and TX GPIO17. Set `SENSORBENCH_ENABLE_GPS=0` to omit GPS.

The firmware was built using ESP32 core 3.3.12, Adafruit BNO08x 1.2.7, Adafruit Unified Sensor and Adafruit BusIO. TinyGPSPlus 1.0.3 is used when GPS is enabled.

```sh
arduino-cli compile --fqbn esp32:esp32:esp32 firmware/SensorBench_BNO085
```

The [telemetry contract](../docs/QUATERNION_PROTOCOL.md) defines the sample and event fields. The firmware reports its requested 100 ms interval and elapsed time using `millis()`. Use these values to distinguish the requested cadence from the observed timestamps. The optional GPS UTC value reflects the latest fresh GPS fix time fields and can repeat across IMU samples.

This sketch has compile verification. The separate physical GPS and DS3231 demonstration used the original `MGRS_kello` firmware and its own capture profile. See [verification](../docs/VERIFICATION.md) for both results.
