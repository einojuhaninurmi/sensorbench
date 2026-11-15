# USB capture and offline replay

SensorBench supports a GPS and RTC text stream and a quaternion JSON stream. Run the commands from the repository root. Install serial support with `python -m pip install ".[serial]"`, or use `python -m pip install ".[gps]"` for serial and MGRS conversion together.

## GPS and DS3231

The existing ESP32 `MGRS_kello` firmware emits a timestamp and decimal-degree coordinates in the format `DD.MM.YYYY HH:MM:SS,latitude,longitude`. The GPS profile preserves the clock's configured `+02:00` offset and accepts repeated timestamps at the firmware's one-second resolution. Clock readings that go backwards fail the time-order check.

```sh
python -m sensorbench replay examples/gps_rtc_demo.txt --device sim-gps --profile gps_monitoring
```

With compatible firmware, find the serial port with `python -m sensorbench ports`. Close other serial monitors using the same port. Replace `COM3` below with the port for the device.

```sh
python -m sensorbench capture --profile gps_monitoring --port COM3 --device esp32-gps-rtc --seconds 15
```

## BNO085 quaternion telemetry

The included [ESP32 firmware](../firmware/README.md) emits one JSON object per line over USB serial at 115200 baud. The [telemetry contract](QUATERNION_PROTOCOL.md) describes its fields. A stationary session uses the `monitoring` profile. Use `guided_motion` for a deliberate rotation exercise.

```sh
python -m sensorbench replay examples/quaternion_demo.ndjson --device sim-quaternion --profile guided_motion
python -m sensorbench capture --port COM3 --device esp32-bno085 --seconds 15
```

The software uses the requested `period_ms` as the timing-check baseline and checks the recorded device times. Start a new session after the controller restarts or its counters wrap.

## Results

Each capture or replay saves a `.raw.log` and a JSON session under `data/captures/` by default. The JSON contains a hash of the original bytes, device and firmware metadata, and the converted records. The original bytes remain available after an acquisition error. The default output directory is Git-ignored and can contain precise physical locations.

Replay labels its input as simulator data by default. Live serial capture labels its data as hardware. The included examples are synthetic.

Use the same database when you open the dashboard. You can also download a report using the run ID printed by the capture command.

```sh
python -m sensorbench serve
python -m sensorbench report RUN_ID --output reports/capture.md
```
