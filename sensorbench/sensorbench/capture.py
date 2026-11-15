"""Read-only quaternion and GPS/RTC telemetry capture and replay.

Only ``capture_serial`` opens a port, and only the explicitly supplied port.
Replay is offline and defaults to a simulator source label. Raw bytes are saved
before parsing; any aborted acquisition retains its log and is not imported.
"""

from __future__ import annotations

from datetime import datetime, timedelta, timezone
import hashlib
from importlib.metadata import version
import math
from pathlib import Path
import re
import time
import uuid

from .orientation import quaternion_to_euler
from .storage import InputError, Store, canonical_json, strict_json

PROTOCOL = "sensorbench-quaternion-v1"
ADAPTER_VERSION = "quaternion-adapter-0.1"
CONVERSION_VERSION = "quaternion-zyx-v1"
GPS_PROTOCOL = "gps-rtc-csv-v1"
GPS_ADAPTER_VERSION = "gps-rtc-adapter-0.1"
GPS_CLOCK_OFFSET = timezone(timedelta(hours=2))
CAPTURE_PROFILES = ("monitoring", "guided_motion", "gps_monitoring")
MAX_LINE_BYTES = 4096
MAX_RAW_BYTES = 64 * 1024 * 1024
MAX_SAMPLES = 20_000
FINAL_FRAME_GRACE_S = 0.5
SERIAL_HINT = "Install optional serial support with: python -m pip install -r requirements-serial.txt"


class CaptureError(InputError):
    """Acquisition or protocol failure; no completed session was imported."""


def _identifier(value, name):
    if not isinstance(value, str) or not value.strip() or len(value) > 120 or not value.isprintable():
        raise CaptureError(f"{name} must be a nonempty printable string of at most 120 characters")
    return value


def _finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    try:
        result = float(value)
    except (ValueError, OverflowError) as exc:
        raise ValueError(f"{name} must be a finite number") from exc
    if not math.isfinite(result):
        raise ValueError(f"{name} must be a finite number")
    return result


def _uint(value, name, maximum=0xFFFFFFFF):
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= maximum:
        raise ValueError(f"{name} must be an integer between 0 and {maximum}")
    return value


def _utc(value):
    if not isinstance(value, str) or len(value) > 64:
        raise ValueError("gps_utc must be a UTC ISO8601 timestamp")
    try:
        parsed = datetime.fromisoformat(value[:-1] + "+00:00" if value.endswith("Z") else value)
    except ValueError as exc:
        raise ValueError("gps_utc must be a valid UTC ISO8601 timestamp") from exc
    if "T" not in value or parsed.tzinfo is None or parsed.utcoffset().total_seconds() != 0:
        raise ValueError("gps_utc must include a UTC timezone (Z or +00:00)")


class LineAdapter:
    def __init__(self):
        self.buffer = bytearray()
        self.line_number = 0

    def feed(self, chunk):
        if not isinstance(chunk, (bytes, bytearray)):
            raise CaptureError("Telemetry transport must return bytes")
        self.buffer.extend(chunk)
        while True:
            end = self.buffer.find(b"\n")
            if end < 0:
                break
            if end > MAX_LINE_BYTES:
                raise CaptureError(f"Telemetry line exceeds {MAX_LINE_BYTES} bytes")
            line = bytes(self.buffer[:end]).rstrip(b"\r")
            del self.buffer[:end + 1]
            self._line(line)
        if len(self.buffer) > MAX_LINE_BYTES:
            raise CaptureError(f"Telemetry line exceeds {MAX_LINE_BYTES} bytes")


class QuaternionAdapter(LineAdapter):
    """Frame strict NDJSON while retaining each original sample packet."""

    def __init__(self):
        super().__init__()
        self.samples = []
        self.stream_started = False
        self.boot_lines_ignored = 0
        self.firmware_version = None
        self.period_ms = None
        self.event_summary = {"ready": 0, "sensor_reset": 0, "error": 0}

    def _line(self, raw):
        self.line_number += 1
        try:
            text = raw.decode("utf-8")
        except UnicodeDecodeError as exc:
            if not self.stream_started:
                self.boot_lines_ignored += 1
                return
            raise CaptureError(f"Non-UTF8 data after telemetry started at line {self.line_number}") from exc
        try:
            packet = strict_json(text)
        except InputError as exc:
            if not self.stream_started and not text.lstrip().startswith(("{", "[")):
                self.boot_lines_ignored += 1
                return
            raise CaptureError(f"Invalid JSON telemetry at line {self.line_number}: {exc}") from exc
        if not isinstance(packet, dict) or packet.get("protocol") != PROTOCOL:
            raise CaptureError(f"Expected a {PROTOCOL} object at line {self.line_number}")
        try:
            canonical_json(packet).encode("utf-8")
        except UnicodeEncodeError as exc:
            raise CaptureError(f"Invalid Unicode scalar in telemetry at line {self.line_number}") from exc
        kind = packet.get("type")
        if kind not in ("sample", "event"):
            raise CaptureError(f"Unknown telemetry packet type at line {self.line_number}")
        self._metadata(packet, required=kind == "sample")
        self.stream_started = True
        if kind == "event":
            event = packet.get("event")
            if event not in self.event_summary:
                raise CaptureError(f"Unknown device event at line {self.line_number}")
            message = packet.get("message", "")
            if not isinstance(message, str) or len(message) > 1000 or (message and not message.isprintable()):
                raise CaptureError(f"Invalid device event message at line {self.line_number}")
            self.event_summary[event] += 1
            if event == "error":
                raise CaptureError("Device reported an error" + (": " + message if message else ""))
            return
        if len(self.samples) >= MAX_SAMPLES:
            raise CaptureError(f"Capture exceeds {MAX_SAMPLES} sample packets; use a shorter session")
        self.samples.append(self._sample(packet))

    def _metadata(self, packet, required):
        if required or "firmware_version" in packet:
            version = _identifier(packet.get("firmware_version"), "firmware_version")
            if self.firmware_version is not None and version != self.firmware_version:
                raise CaptureError("firmware_version changed within one session")
            self.firmware_version = version
        if required or "period_ms" in packet:
            try:
                period = _uint(packet.get("period_ms"), "period_ms", 60_000)
            except ValueError as exc:
                raise CaptureError(str(exc)) from exc
            if period == 0:
                raise CaptureError("period_ms must be positive")
            if self.period_ms is not None and period != self.period_ms:
                raise CaptureError("period_ms changed within one session")
            self.period_ms = period

    def _sample(self, packet):
        sample = {"seq": packet.get("seq"), "t_ms": packet.get("t_ms"),
                  "quaternion": packet.get("quaternion"), "telemetry": packet,
                  "conversion_version": CONVERSION_VERSION, "capture_line": self.line_number}
        errors = []
        for name in ("seq", "t_ms"):
            try:
                _uint(packet.get(name), name)
            except ValueError as exc:
                errors.append(str(exc))
        try:
            _uint(packet.get("sensor_status"), "sensor_status", 3)
        except ValueError as exc:
            errors.append(str(exc))
        for name in ("sensor_status", "gps_enabled", "gps_fix", "latitude_deg", "longitude_deg", "gps_utc"):
            if name in packet:
                sample[name] = packet[name]
        for name in ("gps_enabled", "gps_fix"):
            if not isinstance(packet.get(name), bool):
                errors.append(f"{name} must be a boolean")
        if packet.get("gps_fix") is True and packet.get("gps_enabled") is not True:
            errors.append("gps_fix cannot be true when GPS is disabled")
        for name, low, high in (("latitude_deg", -90, 90), ("longitude_deg", -180, 180)):
            if name in packet or packet.get("gps_fix") is True:
                try:
                    value = _finite(packet.get(name), name)
                    if not low <= value <= high:
                        raise ValueError(f"{name} must be between {low} and {high}")
                except ValueError as exc:
                    errors.append(str(exc))
        if "gps_utc" in packet:
            try:
                _utc(packet["gps_utc"])
            except ValueError as exc:
                errors.append(str(exc))
        try:
            q = packet.get("quaternion")
            if not isinstance(q, dict):
                raise ValueError("quaternion must contain w, x, y and z")
            values = [_finite(q.get(name), "quaternion." + name) for name in ("w", "x", "y", "z")]
            norm = math.hypot(*values)
            if not 0.95 <= norm <= 1.05:
                raise ValueError("quaternion norm must be within 0.05 of 1.0")
            sample.update(quaternion_to_euler(*values))
        except ValueError as exc:
            errors.append(str(exc))
        if errors:
            sample["adapter_error"] = "; ".join(errors)[:1000]
        return sample

    def finish(self):
        if self.buffer:
            raise CaptureError("Input ended with an incomplete line; packets must end with a newline")
        if not self.samples:
            raise CaptureError("No JSON sample packets received. Use the SensorBench JSON firmware; legacy plaintext is not imported")
        return {"firmware_version": self.firmware_version, "period_ms": self.period_ms,
                "samples": self.samples, "boot_lines_ignored": self.boot_lines_ignored,
                "event_summary": dict(self.event_summary),
                "invalid_sample_packets": sum("adapter_error" in sample for sample in self.samples)}


def adapt_recording(raw):
    """Adapt a bounded, newline-terminated recording without any I/O."""
    if not isinstance(raw, (bytes, bytearray)) or len(raw) > MAX_RAW_BYTES:
        raise CaptureError(f"Recording must be bytes and no larger than {MAX_RAW_BYTES} bytes")
    adapter = QuaternionAdapter()
    for offset in range(0, len(raw), 4096):
        adapter.feed(raw[offset:offset + 4096])
    return adapter.finish()


class GpsRtcAdapter(LineAdapter):
    """Read the existing MGRS_kello sketch's local RTC time and GPS fix."""

    def __init__(self):
        super().__init__()
        self.samples = []
        self.boot_lines_ignored = 0
        try:
            from mgrs import MGRS
            from mgrs.core import MGRSError
        except ImportError:
            self.converter = None
            self.converter_version = None
        else:
            self.converter = MGRS()
            self.converter_error = MGRSError
            self.converter_version = version("mgrs")

    def _line(self, raw):
        self.line_number += 1
        text = raw.decode("utf-8", errors="replace")
        if not text.strip():
            return
        if text.count(",") != 2 and not self.samples and not text.startswith("VIRHE"):
            self.boot_lines_ignored += 1
            return
        if len(self.samples) >= MAX_SAMPLES:
            raise CaptureError(f"Capture exceeds {MAX_SAMPLES} sample packets; use a shorter session")
        sample = {"original_line": text, "capture_line": self.line_number}
        try:
            if text.startswith("VIRHE"):
                raise ValueError(text)
            stamp, lat, lon = (part.strip() for part in text.split(","))
            recorded_at = datetime.strptime(stamp, "%d.%m.%Y %H:%M:%S").replace(tzinfo=GPS_CLOCK_OFFSET)
            lat, lon = float(lat), float(lon)
            if not (-90 <= lat <= 90 and -180 <= lon <= 180):
                raise ValueError("latitude or longitude is outside its valid range")
            sample.update(recorded_at=recorded_at.isoformat(), latitude_deg=lat, longitude_deg=lon, gps_fix=True)
            if self.converter is not None:
                try:
                    sample["mgrs"] = self.converter.toMGRS(lat, lon, MGRSPrecision=5)
                except self.converter_error as exc:
                    raise ValueError(f"MGRS conversion failed: {exc}") from exc
        except (ValueError, RuntimeError) as exc:
            sample["adapter_error"] = "Invalid GPS/RTC row: " + str(exc)[:900]
        self.samples.append(sample)

    def finish(self):
        if self.buffer:
            raise CaptureError("Input ended with an incomplete GPS/RTC line")
        if not self.samples:
            raise CaptureError("No GPS/RTC rows received. Wait for a GPS fix and try again")
        return {"firmware_version": "MGRS_kello (build unreported)", "samples": self.samples,
                "boot_lines_ignored": self.boot_lines_ignored,
                "invalid_sample_packets": sum("adapter_error" in sample for sample in self.samples),
                "mgrs_version": self.converter_version}


def _source_profile(device_id, session_id, source, profile):
    _identifier(device_id, "device_id")
    if session_id is not None:
        _identifier(session_id, "session_id")
    if source not in ("simulator", "hardware"):
        raise CaptureError("source must be simulator or hardware")
    if profile not in CAPTURE_PROFILES:
        raise CaptureError("profile must be monitoring, guided_motion or gps_monitoring")


def _paths(output, device_id, transport):
    if output is None:
        slug = re.sub(r"[^A-Za-z0-9_-]+", "-", device_id).strip("-")[:40] or "device"
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        output = Path("data") / "captures" / f"{slug}-{transport}-{stamp}-{uuid.uuid4().hex[:8]}.json"
    output = Path(output)
    raw = output.with_suffix(".raw.log")
    if output == raw:
        raise CaptureError("Payload output and raw sidecar must have distinct paths")
    if output.exists() or raw.exists():
        raise CaptureError("Refusing to overwrite an existing payload or raw log; choose a new --output")
    output.parent.mkdir(parents=True, exist_ok=True)
    return output, raw


def _payload(adapted, device_id, session_id, source, profile, capture):
    if profile == "gps_monitoring":
        capture.update(protocol=GPS_PROTOCOL, adapter_version=GPS_ADAPTER_VERSION,
                       clock_offset="+02:00", boot_lines_ignored=adapted["boot_lines_ignored"],
                       sample_packets=len(adapted["samples"]), invalid_sample_packets=adapted["invalid_sample_packets"])
        if adapted["mgrs_version"]:
            capture.update(coordinate_converter="Python mgrs " + adapted["mgrs_version"], mgrs_digits_per_axis=5)
        return {"device_id": device_id, "session_id": session_id, "firmware_version": adapted["firmware_version"],
                "source": source, "test_profile": profile, "scenario": "gps_rtc_recording",
                "capture": capture, "samples": adapted["samples"]}
    capture.update({"protocol": PROTOCOL, "adapter_version": ADAPTER_VERSION,
                    "conversion_version": CONVERSION_VERSION, "period_ms": adapted["period_ms"],
                    "boot_lines_ignored": adapted["boot_lines_ignored"], "event_summary": adapted["event_summary"],
                    "sample_packets": len(adapted["samples"]), "invalid_sample_packets": adapted["invalid_sample_packets"]})
    return {"device_id": device_id, "session_id": session_id, "firmware_version": adapted["firmware_version"],
            "source": source, "test_profile": profile, "scenario": "quaternion_recording",
            "test_config": {"expected_interval_ms": adapted["period_ms"]},
            "capture": capture, "samples": adapted["samples"]}


def _save_import(payload, output, raw_path, db):
    # Exclusive creation is the final overwrite guard, including races after
    # acquisition. Failed import keeps both artifacts for inspection.
    with output.open("x", encoding="utf-8", newline="\n") as stream:
        stream.write(canonical_json(payload) + "\n")
    result = Store(db).ingest(payload)
    return {**result, "payload_path": str(output), "raw_log_path": str(raw_path)}


def replay_file(input_path, *, device_id, session_id=None, source="simulator", profile="monitoring",
                db="data/sensorbench.sqlite3", output=None):
    """Copy and replay telemetry offline; default session identity is content based."""
    _source_profile(device_id, session_id, source, profile)
    input_path = Path(input_path)
    if input_path.stat().st_size > MAX_RAW_BYTES:
        raise CaptureError(f"Recording exceeds the {MAX_RAW_BYTES}-byte limit")
    output, raw_path = _paths(output, device_id, "replay")
    digest = hashlib.sha256()
    byte_count = 0
    adapter = GpsRtcAdapter() if profile == "gps_monitoring" else QuaternionAdapter()
    try:
        with input_path.open("rb") as source_stream, raw_path.open("xb") as raw_stream:
            while chunk := source_stream.read(4096):
                raw_stream.write(chunk)
                digest.update(chunk)
                byte_count += len(chunk)
                if byte_count > MAX_RAW_BYTES:
                    raise CaptureError(f"Recording exceeds the {MAX_RAW_BYTES}-byte limit")
                adapter.feed(chunk)
        adapted = adapter.finish()
        capture = {"transport": "file-replay", "raw_log_sha256": digest.hexdigest(), "raw_log_bytes": byte_count}
        if session_id is None:
            identity = {"raw_sha256": digest.hexdigest(), "source": source,
                        "profile": profile, "adapter_version": ADAPTER_VERSION}
            if profile == "gps_monitoring":
                identity.update(adapter_version=GPS_ADAPTER_VERSION, mgrs_version=adapted["mgrs_version"])
            identity = canonical_json(identity)
            session_id = "replay-v1-" + hashlib.sha256(identity.encode("utf-8")).hexdigest()[:24]
        payload = _payload(adapted, device_id, session_id, source, profile, capture)
        return _save_import(payload, output, raw_path, db)
    except (Exception, KeyboardInterrupt) as exc:
        raise CaptureError(f"Replay not completed: {exc}. Raw log retained at {raw_path}") from exc


def _serial_module():
    try:
        import serial
    except ImportError as exc:
        raise CaptureError(SERIAL_HINT) from exc
    return serial


def list_ports():
    """Enumerate optional pyserial port metadata without opening any port."""
    _serial_module()
    from serial.tools import list_ports as port_tools
    return [{"port": item.device, "description": item.description, "hwid": item.hwid}
            for item in port_tools.comports()]


def capture_serial(port, *, device_id, seconds=30, session_id=None, profile="monitoring", baudrate=115200,
                   db="data/sensorbench.sqlite3", output=None, serial_factory=None,
                   monotonic=time.monotonic, utcnow=None):
    """Capture one explicitly chosen serial port, save raw evidence, then import.

    Tests inject a fake serial factory and clock. DTR/RTS are disabled before
    ``open``; this does not guarantee that every USB board avoids resetting.
    The adapter never writes serial commands. Timeout reads are at most 0.25 s.
    If the deadline cuts a frame, up to 0.5 s is reserved to finish that line.
    """
    _source_profile(device_id, session_id, "hardware", profile)
    _identifier(port, "port")
    try:
        seconds = _finite(seconds, "seconds")
    except ValueError as exc:
        raise CaptureError(str(exc)) from exc
    if not 1 <= seconds <= 3600:
        raise CaptureError("seconds must be between 1 and 3600")
    if isinstance(baudrate, bool) or not isinstance(baudrate, int) or not 1200 <= baudrate <= 4_000_000:
        raise CaptureError("baudrate must be an integer between 1200 and 4000000")
    if serial_factory is None:
        serial_factory = _serial_module().Serial
    utcnow = utcnow or (lambda: datetime.now(timezone.utc))
    output, raw_path = _paths(output, device_id, "serial")
    digest = hashlib.sha256()
    byte_count = 0
    adapter = GpsRtcAdapter() if profile == "gps_monitoring" else QuaternionAdapter()
    serial_port = None
    started_at = utcnow().isoformat(timespec="milliseconds")
    start = monotonic()
    deadline = start + seconds
    try:
        with raw_path.open("xb") as raw_stream:
            try:
                serial_port = serial_factory(port=None, baudrate=baudrate, timeout=min(0.25, seconds), write_timeout=0.25)
                serial_port.dtr = False
                serial_port.rts = False
                serial_port.port = port
                serial_port.open()
                def read_and_record(size, remaining):
                    nonlocal byte_count
                    serial_port.timeout = min(0.25, remaining)
                    chunk = serial_port.read(size)
                    if not isinstance(chunk, (bytes, bytearray)):
                        raise CaptureError("Serial read must return bytes")
                    if not chunk:
                        return
                    raw_stream.write(chunk)
                    digest.update(chunk)
                    byte_count += len(chunk)
                    if byte_count > MAX_RAW_BYTES:
                        raise CaptureError(f"Raw capture exceeds {MAX_RAW_BYTES} bytes")
                    adapter.feed(chunk)
                while (remaining := deadline - monotonic()) > 0:
                    read_and_record(4096, remaining)
                # Finish only the line already in progress. A larger read here
                # could also consume the beginning of the following packet.
                while adapter.buffer and (remaining := deadline + FINAL_FRAME_GRACE_S - monotonic()) > 0:
                    read_and_record(1, remaining)
            finally:
                if serial_port is not None:
                    serial_port.close()
        duration = monotonic() - start
        adapted = adapter.finish()
        session_id = session_id or "capture-" + uuid.uuid4().hex
        capture = {"transport": "serial", "raw_log_sha256": digest.hexdigest(), "raw_log_bytes": byte_count,
                   "port": port, "baudrate": baudrate, "host_started_at": started_at,
                   "host_ended_at": utcnow().isoformat(timespec="milliseconds"), "host_duration_s": duration,
                   "requested_duration_s": seconds, "final_frame_grace_limit_s": FINAL_FRAME_GRACE_S}
        payload = _payload(adapted, device_id, session_id, "hardware", profile, capture)
        return _save_import(payload, output, raw_path, db)
    except (Exception, KeyboardInterrupt) as exc:
        raise CaptureError(f"Capture not completed: {exc}. Raw log retained at {raw_path}") from exc
