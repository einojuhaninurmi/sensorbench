# Verification

| Input or test | Observed result |
| --- | --- |
| Physical ESP32 GPS and DS3231 capture | 11 of 11 records valid, 11 MGRS references, five GPS checks passed |
| Synthetic quaternion replay | Six records, eight guided-motion checks passed |
| Synthetic orientation scenarios | Normal motion passed. Packet loss, frozen motion and an out-of-range angle failed their expected checks. |
| Combined BNO085/GPS firmware | Compile succeeded with ESP32 core 3.3.12, target `esp32:esp32:esp32`. Build reported 309,232 bytes of program storage and 26,772 bytes of global RAM. |

The GPS capture hash matches its paired raw recording. The physical recordings and SQLite database are private. The included screenshot and fixtures use synthetic input.

The hardware session used the existing GPS and DS3231 firmware. Evidence for the combined BNO085/GPS sketch consists of the build result and software tests of the quaternion receiver. A calibrated reference would be needed to evaluate absolute sensor accuracy.

## Reproducing the demo

```sh
python -m unittest discover -s tests -v
python -m sensorbench demo
python -m sensorbench replay examples/quaternion_demo.ndjson --device sim-quaternion --profile guided_motion
python -m sensorbench replay examples/gps_rtc_demo.txt --device sim-gps --profile gps_monitoring
```

Automated tests cover validation, missing sequences and timing gaps, transport framing, quaternion conversion, replay conflicts, transactional rollback and HTTP ingestion. Serial tests use a test transport. The GitHub Actions matrix is configured for Windows and Ubuntu.
