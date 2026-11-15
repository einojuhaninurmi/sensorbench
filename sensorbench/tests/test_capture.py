"""Capture tests use files, fake clocks and fake serial ports only."""

from contextlib import redirect_stderr, redirect_stdout
from datetime import datetime, timedelta, timezone
import hashlib
import io
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from sensorbench import __main__ as cli
from sensorbench.capture import (ADAPTER_VERSION, CONVERSION_VERSION, PROTOCOL, CaptureError,
                                 QuaternionAdapter, adapt_recording, capture_serial, replay_file)
from sensorbench.storage import Store, strict_json


def packet(seq=0, **changes):
    angle = math.radians(seq * 10) / 2
    result = {"type": "sample", "protocol": PROTOCOL, "firmware_version": "test-fw-1",
              "seq": seq, "t_ms": seq * 100, "period_ms": 100,
              "quaternion": {"w": math.cos(angle), "x": 0, "y": 0, "z": math.sin(angle)},
              "sensor_status": 3, "gps_enabled": False, "gps_fix": False}
    result.update(changes)
    return result


def line(value):
    return (json.dumps(value, allow_nan=False) + "\n").encode("utf-8")


def recording(count=6):
    return b"".join(line(packet(i)) for i in range(count))


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def utcnow(self):
        return datetime(2026, 11, 6, 12, tzinfo=timezone.utc) + timedelta(seconds=self.now)


class FakeSerial:
    """Never touches hardware. A write is a test failure."""

    def __init__(self, chunks, clock, open_error=None):
        self.chunks = list(chunks)
        self.clock = clock
        self.open_error = open_error
        self.dtr = True
        self.rts = True
        self.port = None
        self.closed = False
        self.opened = False
        self.writes = 0
        self.reads = 0
        self.options = None
        self.timeout = None
        self.read_sizes = []

    def factory(self, **options):
        self.options = options
        self.timeout = options["timeout"]
        if options["port"] is not None:
            raise AssertionError("Port must not be opened by construction")
        return self

    def open(self):
        if self.dtr or self.rts:
            raise AssertionError("DTR and RTS must be false before opening")
        self.opened = True
        if self.open_error:
            raise self.open_error

    def read(self, size):
        self.reads += 1
        self.read_sizes.append(size)
        if size not in (4096, 1):
            raise AssertionError("Unexpected serial read bound")
        if self.chunks:
            self.clock.now += 0.001
            result = self.chunks.pop(0)
            if isinstance(result, BaseException):
                raise result
            if len(result) > size:
                self.chunks.insert(0, result[size:])
            return result[:size]
        self.clock.now += self.timeout
        return b""

    def write(self, value):
        self.writes += 1
        raise AssertionError("Capture must never write serial commands")

    def close(self):
        self.closed = True


class DeadlineSerial(FakeSerial):
    def read(self, size):
        # Idle until the last main-window read, then deliver its chunk exactly
        # at the deadline. Further bytes are available only to the final drain.
        if self.reads < 3:
            self.reads += 1
            self.read_sizes.append(size)
            self.clock.now += self.timeout
            return b""
        at_deadline = self.reads == 3
        chunk = super().read(size)
        if at_deadline:
            self.clock.now = 1.0
        return chunk


class AdapterTests(unittest.TestCase):
    def test_original_quaternion_and_packet_are_retained(self):
        original = packet(9, t_ms=12345)
        original["seq"] = 91
        sample = adapt_recording(line(original))["samples"][0]
        self.assertEqual(sample["seq"], 91)
        self.assertEqual(sample["t_ms"], 12345)
        self.assertEqual(sample["quaternion"], original["quaternion"])
        self.assertEqual(sample["telemetry"], original)
        self.assertEqual(sample["conversion_version"], CONVERSION_VERSION)
        self.assertAlmostEqual(sample["yaw_deg"], 90)
        self.assertNotIn("adapter_error", sample)

    def test_partial_reads_and_startup_text(self):
        raw = b"ESP-ROM: boot message\r\n" + line({"type": "event", "protocol": PROTOCOL, "event": "ready"}) + recording(2)
        adapter = QuaternionAdapter()
        for offset in range(0, len(raw), 7):
            adapter.feed(raw[offset:offset + 7])
        result = adapter.finish()
        self.assertEqual(result["boot_lines_ignored"], 1)
        self.assertEqual(result["event_summary"]["ready"], 1)
        self.assertEqual([sample["seq"] for sample in result["samples"]], [0, 1])

    def test_bad_quaternion_is_an_evidence_row_not_dropped(self):
        bad = packet(quaternion={"w": 2, "x": 0, "y": 0, "z": 0})
        result = adapt_recording(line(bad) + line(packet(1)))
        self.assertEqual(len(result["samples"]), 2)
        self.assertEqual(result["invalid_sample_packets"], 1)
        self.assertIn("norm", result["samples"][0]["adapter_error"])
        self.assertNotIn("roll_deg", result["samples"][0])
        self.assertEqual(result["samples"][0]["telemetry"], bad)

    def test_norm_tolerance_boundaries(self):
        for norm in (0.95, 1.05):
            with self.subTest(norm=norm):
                result = adapt_recording(line(packet(quaternion={"w": norm, "x": 0, "y": 0, "z": 0})))
                self.assertNotIn("adapter_error", result["samples"][0])
        for norm in (0, 0.94999, 1.05001):
            with self.subTest(norm=norm):
                result = adapt_recording(line(packet(quaternion={"w": norm, "x": 0, "y": 0, "z": 0})))
                self.assertIn("adapter_error", result["samples"][0])

    def test_nonfinite_string_component_is_invalid_evidence(self):
        result = adapt_recording(line(packet(quaternion={"w": "NaN", "x": 0, "y": 0, "z": 0})))
        self.assertIn("finite", result["samples"][0]["adapter_error"])
        json.dumps(result, allow_nan=False)

    def test_utc_requires_valid_utc_timezone(self):
        for value in ("2026-11-06T12:00:00Z", "2026-11-06T12:00:00.123+00:00"):
            self.assertNotIn("adapter_error", adapt_recording(line(packet(gps_utc=value)))["samples"][0])
        for value in ("2026-11-06", "2026-11-06T12:00:00", "2026-11-06T12:00:00+02:00", "2026-02-30T12:00:00Z", None):
            with self.subTest(value=value):
                self.assertIn("gps_utc", adapt_recording(line(packet(gps_utc=value)))["samples"][0]["adapter_error"])

    def test_packet_values_are_never_invented_or_resequenced(self):
        bad = packet(seq=-1, t_ms="missing", sensor_status=True, gps_enabled=False, gps_fix=True)
        sample = adapt_recording(line(bad))["samples"][0]
        self.assertEqual(sample["seq"], -1)
        self.assertEqual(sample["t_ms"], "missing")
        self.assertIn("seq", sample["adapter_error"])
        self.assertIn("t_ms", sample["adapter_error"])
        self.assertIn("sensor_status", sample["adapter_error"])
        self.assertIn("GPS is disabled", sample["adapter_error"])

    def test_gps_fix_requires_real_valid_coordinates(self):
        valid = packet(gps_enabled=True, gps_fix=True, latitude_deg=60.17, longitude_deg=24.94)
        self.assertNotIn("adapter_error", adapt_recording(line(valid))["samples"][0])
        invalid = packet(gps_enabled=True, gps_fix=True, latitude_deg=120)
        error = adapt_recording(line(invalid))["samples"][0]["adapter_error"]
        self.assertIn("latitude_deg", error)
        self.assertIn("longitude_deg", error)

    def test_metadata_changes_abort(self):
        for changed in (packet(1, firmware_version="other"), packet(1, period_ms=200)):
            with self.subTest(changed=changed):
                with self.assertRaisesRegex(CaptureError, "changed within"):
                    adapt_recording(line(packet()) + line(changed))

    def test_error_event_aborts_but_sensor_reset_is_recorded(self):
        reset = {"type": "event", "protocol": PROTOCOL, "event": "sensor_reset"}
        result = adapt_recording(line(packet()) + line(reset) + line(packet(1)))
        self.assertEqual(result["event_summary"]["sensor_reset"], 1)
        error = {"type": "event", "protocol": PROTOCOL, "event": "error", "message": "I2C failure"}
        with self.assertRaisesRegex(CaptureError, "I2C failure"):
            adapt_recording(line(packet()) + line(error))

    def test_plaintext_after_ready_is_not_silently_dropped(self):
        ready = {"type": "event", "protocol": PROTOCOL, "event": "ready"}
        with self.assertRaisesRegex(CaptureError, "Invalid JSON telemetry"):
            adapt_recording(line(ready) + b"debug text\n" + recording(2))

    def test_strict_json_and_utf8(self):
        for raw in (b'{"protocol":"x","protocol":"y"}\n', b'{"q":NaN}\n', b'{"q":Infinity}\n'):
            with self.subTest(raw=raw):
                with self.assertRaises(CaptureError):
                    adapt_recording(raw)
        with self.assertRaisesRegex(CaptureError, "Non-UTF8"):
            adapt_recording(line(packet()) + b"\xff\n")

    def test_limits_incomplete_lines_and_legacy_only_input(self):
        with self.assertRaisesRegex(CaptureError, "exceeds"):
            adapt_recording(b"x" * 4097)
        with self.assertRaisesRegex(CaptureError, "incomplete"):
            adapt_recording(line(packet()).rstrip(b"\n"))
        with self.assertRaisesRegex(CaptureError, "legacy plaintext"):
            adapt_recording(b"BNO085 ready\n0.1,0.2,0.3,0.4\n")
        with patch("sensorbench.capture.MAX_SAMPLES", 2):
            with self.assertRaisesRegex(CaptureError, "sample packets"):
                adapt_recording(recording(3))


class RecordingTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.db = self.root / "test.sqlite3"

    def tearDown(self):
        self.temp.cleanup()

    def replay(self, raw=None, name="capture.json", **kwargs):
        source = self.root / "input.ndjson"
        source.write_bytes(recording() if raw is None else raw)
        return replay_file(source, device_id="test-device", db=self.db, output=self.root / name, **kwargs)

    def test_replay_provenance_default_source_and_idempotence(self):
        raw = b"boot text\n" + recording()
        first = self.replay(raw)
        second = self.replay(raw, name="second.json")
        self.assertTrue(first["created"])
        self.assertFalse(second["created"])
        self.assertEqual(first["run"]["run_id"], second["run"]["run_id"])
        payload = strict_json(Path(first["payload_path"]).read_text(encoding="utf-8"))
        self.assertEqual(payload["source"], "simulator")
        self.assertEqual(payload["test_profile"], "monitoring")
        self.assertEqual(payload["test_config"]["expected_interval_ms"], 100)
        self.assertEqual(payload["capture"]["adapter_version"], ADAPTER_VERSION)
        self.assertEqual(payload["capture"]["raw_log_sha256"], hashlib.sha256(raw).hexdigest())
        self.assertEqual(payload["capture"]["raw_log_bytes"], len(raw))
        self.assertEqual(payload["capture"]["boot_lines_ignored"], 1)
        self.assertEqual(Path(first["raw_log_path"]).read_bytes(), raw)
        self.assertEqual(first["run"]["capture"], payload["capture"])

    def test_gps_rtc_replay_preserves_time_and_failed_raw_row(self):
        raw = (b"Starting GPS/RTC\r\n"
               b"07.11.2026 22:47:10,60.000000,25.000000\r\n"
               b"07.11.2026 22:47:11,invalid,25.000000\r\n"
               b"07.11.2026 22:47:12,60.000010,25.000000\r\n")
        result = self.replay(raw, profile="gps_monitoring")
        detail = Store(self.db).get_run(result["run"]["run_id"])
        rows = detail["analysis"]["rows"]
        self.assertEqual(detail["run"]["valid_count"], 2)
        self.assertEqual(detail["run"]["invalid_count"], 1)
        self.assertEqual(rows[0]["sample"]["recorded_at"], "2026-11-07T22:47:10+02:00")
        self.assertNotIn("seq", rows[0]["sample"])
        self.assertIn("invalid", rows[1]["raw"]["original_line"])
        self.assertEqual(Path(result["raw_log_path"]).read_bytes(), raw)

    def test_gps_rtc_serial_uses_same_adapter_on_fragmented_input(self):
        raw = (b"07.11.2026 22:47:10,60.000000,25.000000\r\n"
               b"07.11.2026 22:47:11,60.000010,25.000000\r\n")
        clock = FakeClock()
        fake = FakeSerial([raw[:13], raw[13:]], clock)
        result = self.capture(fake, clock, profile="gps_monitoring")
        self.assertEqual(result["run"]["status"], "pass")
        self.assertEqual(result["run"]["source"], "hardware")
        self.assertEqual(result["run"]["capture"]["protocol"], "gps-rtc-csv-v1")
        self.assertEqual(fake.writes, 0)
        self.assertTrue(fake.closed)

    def test_bad_quaternion_import_is_failed_not_successfully_discarded(self):
        raw = line(packet(quaternion={"w": 2, "x": 0, "y": 0, "z": 0})) + recording(2)
        result = self.replay(raw)
        detail = Store(self.db).get_run(result["run"]["run_id"])
        self.assertEqual(detail["run"]["status"], "fail")
        self.assertEqual(detail["run"]["sample_count"], 3)
        self.assertIn("adapter_error", detail["analysis"]["rows"][0]["raw"])

    def test_outputs_are_never_overwritten(self):
        destination = self.root / "capture.json"
        destination.write_text("existing", encoding="utf-8")
        with self.assertRaisesRegex(CaptureError, "overwrite"):
            self.replay()
        self.assertEqual(destination.read_text(encoding="utf-8"), "existing")
        destination.unlink()
        raw = self.root / "capture.raw.log"
        raw.write_bytes(b"existing raw")
        with self.assertRaisesRegex(CaptureError, "overwrite"):
            self.replay()
        self.assertEqual(raw.read_bytes(), b"existing raw")
        self.assertFalse(destination.exists())

    def test_replay_failure_keeps_raw_and_does_not_import(self):
        raw = line(packet()) + b"unexpected debug output\n"
        with self.assertRaisesRegex(CaptureError, "Raw log retained"):
            self.replay(raw)
        self.assertEqual((self.root / "capture.raw.log").read_bytes(), raw)
        self.assertFalse((self.root / "capture.json").exists())
        self.assertFalse(self.db.exists())

    def capture(self, fake, clock, name="serial.json", **kwargs):
        return capture_serial("COM-FAKE", device_id="test-device", seconds=1, db=self.db,
                              output=self.root / name, serial_factory=fake.factory,
                              monotonic=clock, utcnow=clock.utcnow, **kwargs)

    def test_fake_serial_partial_reads_close_and_provenance(self):
        raw = b"ESP boot\n" + recording()
        clock = FakeClock()
        fake = FakeSerial([raw[i:i + 17] for i in range(0, len(raw), 17)], clock)
        result = self.capture(fake, clock)
        self.assertTrue(fake.opened)
        self.assertTrue(fake.closed)
        self.assertEqual(fake.writes, 0)
        self.assertFalse(fake.dtr)
        self.assertFalse(fake.rts)
        self.assertEqual(fake.port, "COM-FAKE")
        self.assertLessEqual(fake.options["timeout"], .25)
        payload = strict_json(Path(result["payload_path"]).read_text(encoding="utf-8"))
        self.assertEqual(payload["source"], "hardware")
        self.assertEqual(payload["capture"]["transport"], "serial")
        self.assertEqual(payload["capture"]["port"], "COM-FAKE")
        self.assertEqual(payload["capture"]["raw_log_sha256"], hashlib.sha256(raw).hexdigest())
        self.assertGreaterEqual(payload["capture"]["host_duration_s"], 1)
        self.assertLess(payload["capture"]["host_duration_s"], 1.3)
        self.assertEqual(Path(result["raw_log_path"]).read_bytes(), raw)

    def test_empty_serial_timeout_is_bounded_and_closes(self):
        clock = FakeClock()
        fake = FakeSerial([], clock)
        with self.assertRaisesRegex(CaptureError, "No JSON sample"):
            self.capture(fake, clock)
        self.assertTrue(fake.closed)
        self.assertLessEqual(fake.reads, 5)
        self.assertFalse((self.root / "serial.json").exists())
        self.assertEqual((self.root / "serial.raw.log").read_bytes(), b"")
        self.assertFalse(self.db.exists())

    def test_deadline_completes_current_frame_without_reading_next_frame(self):
        complete = recording(2)
        clock = FakeClock()
        fake = DeadlineSerial([complete[:-8], complete[-8:] + line(packet(2))], clock)
        result = self.capture(fake, clock)
        self.assertEqual(result["run"]["status"], "pass")
        self.assertEqual(result["run"]["sample_count"], 2)
        self.assertEqual(Path(result["raw_log_path"]).read_bytes(), complete)
        self.assertEqual(fake.read_sizes[4:], [1] * 8)
        self.assertLessEqual(clock.now, 1.5)
        self.assertTrue(fake.closed)

    def test_deadline_with_unfinished_frame_keeps_raw_and_does_not_import(self):
        partial = recording(2)[:-8]
        clock = FakeClock()
        fake = DeadlineSerial([partial], clock)
        with self.assertRaisesRegex(CaptureError, "incomplete.*Raw log retained"):
            self.capture(fake, clock)
        self.assertAlmostEqual(clock.now, 1.5)
        self.assertTrue(fake.closed)
        self.assertEqual((self.root / "serial.raw.log").read_bytes(), partial)
        self.assertFalse(self.db.exists())

    def test_serial_errors_always_close_and_keep_raw(self):
        error_event = line({"type": "event", "protocol": PROTOCOL, "event": "error", "message": "sensor lost"})
        for index, tail in enumerate((error_event, b"unexpected text\n", OSError("USB disconnected"))):
            with self.subTest(tail=tail):
                clock = FakeClock()
                fake = FakeSerial([line(packet()), tail], clock)
                with self.assertRaisesRegex(CaptureError, "Raw log retained"):
                    self.capture(fake, clock, name=f"bad-{index}.json")
                self.assertTrue(fake.closed)
                self.assertEqual(fake.writes, 0)
                self.assertFalse((self.root / f"bad-{index}.json").exists())
                self.assertTrue((self.root / f"bad-{index}.raw.log").read_bytes().startswith(line(packet())))
        self.assertFalse(self.db.exists())

    def test_open_failure_closes_and_preserves_empty_log(self):
        clock = FakeClock()
        fake = FakeSerial([], clock, OSError("port busy"))
        with self.assertRaisesRegex(CaptureError, "port busy"):
            self.capture(fake, clock)
        self.assertTrue(fake.closed)
        self.assertFalse((self.root / "serial.json").exists())
        self.assertTrue((self.root / "serial.raw.log").exists())

    def test_invalid_duration_and_existing_output_do_not_open_port(self):
        for seconds in (0, 3601, float("nan"), True):
            fake = FakeSerial([], FakeClock())
            with self.subTest(seconds=seconds):
                with self.assertRaises(CaptureError):
                    capture_serial("COM-FAKE", device_id="test-device", seconds=seconds,
                                   output=self.root / "unused.json", serial_factory=fake.factory)
                self.assertFalse(fake.opened)
        (self.root / "exists.json").write_text("keep", encoding="utf-8")
        fake = FakeSerial([], FakeClock())
        with self.assertRaisesRegex(CaptureError, "overwrite"):
            capture_serial("COM-FAKE", device_id="test-device", output=self.root / "exists.json", serial_factory=fake.factory)
        self.assertFalse(fake.opened)


class CaptureCliTests(unittest.TestCase):
    def test_replay_default_source_and_profile(self):
        with patch("sensorbench.__main__.replay_file", return_value={"created": True}) as replay, redirect_stdout(io.StringIO()):
            self.assertEqual(cli.main(["replay", "sample.ndjson", "--device", "imu-1"]), 0)
        self.assertEqual(replay.call_args.kwargs["source"], "simulator")
        self.assertEqual(replay.call_args.kwargs["profile"], "monitoring")

    def test_capture_requires_explicit_port(self):
        with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as result:
            cli.main(["capture", "--device", "imu-1"])
        self.assertEqual(result.exception.code, 2)

    def test_capture_cli_forwards_settings_without_opening_hardware(self):
        with patch("sensorbench.__main__.capture_serial", return_value={"created": True}) as capture, redirect_stdout(io.StringIO()):
            cli.main(["capture", "--port", "COM-FAKE", "--device", "imu-1", "--seconds", "2", "--session", "s1"])
        self.assertEqual(capture.call_args.args, ("COM-FAKE",))
        self.assertEqual(capture.call_args.kwargs["seconds"], 2)
        self.assertEqual(capture.call_args.kwargs["profile"], "monitoring")

    def test_missing_optional_dependency_has_install_instruction(self):
        error = io.StringIO()
        with patch("sensorbench.__main__.list_ports", side_effect=CaptureError("Install optional serial support with: python -m pip install -r requirements-serial.txt")), redirect_stderr(error), self.assertRaises(SystemExit) as result:
            cli.main(["ports"])
        self.assertEqual(result.exception.code, 2)
        self.assertIn("requirements-serial.txt", error.getvalue())


if __name__ == "__main__":
    unittest.main()
