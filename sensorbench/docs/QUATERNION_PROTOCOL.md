# Quaternion telemetry contract

Protocol: `sensorbench-quaternion-v1`. Firmware: `sensorbench-esp32-0.2.0`.

The combined firmware targets a classic ESP32 WROOM-32D with BNO085 on the default I2C bus and GPS on UART2. Computer-facing USB serial is 115200 baud. GPS UART is 9600 baud, RX16/TX17. The separate GPS and DS3231 stream has its own [capture profile](USB_CAPTURE.md).

## Sample frames

One UTF-8 JSON object is emitted per line. These values illustrate the format; they are not a physical measurement:

```json
{"type":"sample","protocol":"sensorbench-quaternion-v1","firmware_version":"sensorbench-esp32-0.2.0","seq":12,"t_ms":1500,"period_ms":100,"quaternion":{"w":0.923880,"x":0.000000,"y":0.000000,"z":0.382683},"sensor_status":3,"gps_enabled":true,"gps_fix":false}
```

- Quaternion order is scalar-first **w, x, y, z**. Adafruit names these **real, i, j, k**. The user's earlier text output placed i,j,k before real, so positional copying of that order into a w,x,y,z function would be incorrect.
- Components are printed to six decimal places. The adapter preserves the packet and normalizes a finite, nonzero quaternion after checking its norm against the unit-quaternion tolerance. Normalization does not recover precision discarded by an earlier two-decimal printout.
- Conversion uses right-handed active rotations, `R = Rz(yaw) Ry(pitch) Rx(roll)`. Outputs are in degrees. Pitch is between -90 and 90; roll/yaw wrap at 180 degrees. At pitch +/-90, Euler roll/yaw are not uniquely identifiable; the implementation uses a documented stable representation.
- `seq` and `t_ms` originate on the controller. The host does not reconstruct elapsed device time from a row index or assume every packet arrived.
- `period_ms` is the requested 100 ms interval. Timing checks use the actual `t_ms` values and recorded configuration. A successful software conversion does not prove the sensor or its mounting is calibrated.
- `sensor_status` is the sensor's low two status bits. It is not a percentage accuracy score.

## GPS and clock fields

When `gps_fix` is true, fresh latitude/longitude in decimal degrees may be included. With no fresh fix, the coordinates are absent rather than replaced with a guessed location.

`gps_utc` is optional. It is the latest valid, fresh GPS date/time snapshot, independently of position-fix availability. It can repeat between GPS updates and is **not a precisely synchronized UTC timestamp for each IMU sample**. The authoritative interval measurement in this version is the controller's elapsed `t_ms`.

## Events and framing

The sketch emits `ready`, `sensor_reset` and `error` event frames. A sensor reset triggers re-enabling of rotation-vector reports; a controller reboot requires a new capture because counters restart.

ESP32 boot text and an upstream library diagnostic can appear outside JSON. Startup text is counted and retained in the raw log. Non-JSON data after the telemetry stream starts is an explicit acquisition failure, and device error events are reported. Invalid quaternion or clock values in a recognized sample become failed rows with `adapter_error` evidence; they are not silently converted into plausible angles.

USB reads may return a partial JSON line. The receiver buffers fragments up to a bounded line length and uses timeouts. Original raw bytes and their SHA-256 remain separate evidence from the converted session payload. Raw logs can contain actual locations; generated capture files remain in the ignored `data` directory.

## Replay and physical evidence

`examples/quaternion_demo.ndjson` is a synthetic fixture. Replay uses the simulator source label by default. A successful fixture replay verifies framing, conversion, storage and test logic, while a physical USB acquisition is a separate verification step.

For the source naming and conversion convention, see the [official Adafruit quaternion example](https://github.com/adafruit/Adafruit_BNO08x/blob/master/examples/quaternion_yaw_pitch_roll/quaternion_yaw_pitch_roll.ino). For serial timeout and DTR/RTS behavior, see the [pySerial API](https://pyserial.readthedocs.io/en/latest/pyserial_api.html). Setting control lines before open reduces avoidable changes, but an operating-system driver can still toggle them when opening a port; opening a serial port is not a guarantee of preserving MCU uptime.
