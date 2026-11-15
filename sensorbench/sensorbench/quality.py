"""Deterministic quality checks for sensor records, not a calibrated accuracy test.

Angles are degrees: roll [-180, 180], pitch [-90, 90], yaw [-180, 360].
The yaw range accepts common signed and unsigned Euler conventions. Roll and
yaw motion spans use the smallest covering circular arc, so crossing their
wraparound is not mistaken for a large movement. Guided motion requires the
operator or fixture to exercise the device; stationary monitoring skips this
check. At least two valid records are required for any overall pass.
Sequence values are restricted to nonnegative signed 64-bit integers so every
normalized record fits the storage representation.

``raw`` preserves valid JSON input. Nonfinite or non-JSON Python values are
represented by explicit strings so even malformed direct-call input can be
reported with strict JSON serialization. No records are reordered or repaired.
"""

from __future__ import annotations

from datetime import datetime
import math
import re
from typing import Any

DEFAULT_CONFIG = {
    "expected_interval_ms": 20.0,
    "gap_tolerance_factor": 3.0,
    "motion_span_deg": 2.0,
}
TEST_VERSION = "sensor-quality-0.2"
GPS_TEST_VERSION = "sensor-gps-quality-0.1"
TEST_PROFILES = ("guided_motion", "monitoring", "gps_monitoring")
_ANGLE_BOUNDS = {
    "roll_deg": (-180.0, 180.0),
    "pitch_deg": (-90.0, 90.0),
    "yaw_deg": (-180.0, 360.0),
}
_GPS_BOUNDS = {"latitude_deg": (-90.0, 90.0), "longitude_deg": (-180.0, 180.0)}
MAX_SEQUENCE = (1 << 63) - 1
_GPS_RECORDED_AT = re.compile(r"[0-9]{4}-[0-9]{2}-[0-9]{2}T[0-9]{2}:[0-9]{2}:[0-9]{2}(?:\.[0-9]{1,6})?\+02:00")
_MGRS = re.compile(r"(?:(?:0?[1-9]|[1-5][0-9]|60)[C-HJ-NP-X][A-HJ-NP-Z][A-HJ-NP-V]|[ABYZ][A-HJ-NP-Z]{2})[0-9]{10}")


def _number(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    try:
        result = float(value)
    except (ValueError, OverflowError):
        return None
    return result if math.isfinite(result) else None


def _safe_raw(value: Any, seen: set[int] | None = None, depth: int = 0) -> Any:
    """Preserve JSON values; make non-JSON direct-call input reportable."""
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        if math.isfinite(value):
            return value
        return "NaN" if math.isnan(value) else ("Infinity" if value > 0 else "-Infinity")
    if depth >= 100:
        return "<nesting limit>"
    if not isinstance(value, (dict, list, tuple)):
        return f"<unsupported {type(value).__name__}>"
    seen = set() if seen is None else seen
    identity = id(value)
    if identity in seen:
        return "<circular reference>"
    seen.add(identity)
    try:
        if isinstance(value, dict):
            return {str(k): _safe_raw(v, seen, depth + 1) for k, v in value.items()}
        return [_safe_raw(v, seen, depth + 1) for v in value]
    finally:
        seen.remove(identity)


def _settings(config: dict | None) -> dict:
    if config is not None and not isinstance(config, dict):
        raise ValueError("test_config must be an object")
    supplied = {} if config is None else config
    unknown = set(supplied) - set(DEFAULT_CONFIG)
    if unknown:
        raise ValueError("Unknown test_config setting: " + ", ".join(sorted(map(str, unknown))))
    effective = dict(DEFAULT_CONFIG)
    for key, value in supplied.items():
        number = _number(value)
        if number is None:
            raise ValueError(f"{key} must be a finite number, not a boolean")
        effective[key] = number
    if effective["expected_interval_ms"] <= 0:
        raise ValueError("expected_interval_ms must be greater than zero")
    if effective["gap_tolerance_factor"] < 1:
        raise ValueError("gap_tolerance_factor must be at least one")
    if not 0 < effective["motion_span_deg"] <= 360:
        raise ValueError("motion_span_deg must be greater than zero and at most 360")
    if not math.isfinite(effective["expected_interval_ms"] * effective["gap_tolerance_factor"]):
        raise ValueError("The configured maximum sampling gap must be finite")
    return effective


def _circular_span(values: list[float]) -> float:
    if len(values) < 2:
        return 0.0
    angles = sorted(value % 360.0 for value in values)
    gaps = [right - left for left, right in zip(angles, angles[1:])]
    gaps.append(angles[0] + 360.0 - angles[-1])
    return max(0.0, 360.0 - max(gaps))


def _adapter_failure(raw: dict) -> str | None:
    if "adapter_error" not in raw:
        return None
    message = raw["adapter_error"]
    if not isinstance(message, str) or not message.strip() or len(message) > 1000:
        return "adapter_error must be a nonempty string of at most 1000 characters"
    return "telemetry adapter: " + message


def _analyze_gps_samples(samples: list) -> dict:
    rows = []
    schema_indices, gps_indices, mgrs_indices = set(), set(), set()
    bad_clock_indices, order_indices = set(), set()
    previous_clock = None
    backward_steps = 0
    mgrs_present = False

    for index, raw in enumerate(samples):
        errors = []
        normalized = {}
        clock = None
        if not isinstance(raw, dict):
            errors.append("sample must be an object")
        else:
            adapter_error = _adapter_failure(raw)
            if adapter_error:
                errors.append(adapter_error)

            recorded_at = raw.get("recorded_at")
            if isinstance(recorded_at, str) and _GPS_RECORDED_AT.fullmatch(recorded_at):
                try:
                    clock = datetime.fromisoformat(recorded_at)
                except ValueError:
                    pass
            if clock is None:
                errors.append("recorded_at must be a valid ISO 8601 date and time with offset +02:00")
                bad_clock_indices.add(index)
            else:
                normalized["recorded_at"] = recorded_at

            if raw.get("gps_fix") is not True:
                errors.append("gps_fix must be true for a recorded GPS fix")
                gps_indices.add(index)
            else:
                normalized["gps_fix"] = True
            for field, (low, high) in _GPS_BOUNDS.items():
                coordinate = _number(raw.get(field))
                if coordinate is None or not low <= coordinate <= high:
                    errors.append(f"{field} must be a finite number between {low:g} and {high:g}")
                    gps_indices.add(index)
                else:
                    normalized[field] = coordinate

            if "mgrs" in raw:
                mgrs_present = True
                mgrs = raw["mgrs"]
                if not isinstance(mgrs, str) or not _MGRS.fullmatch(mgrs):
                    errors.append("mgrs must be a compact uppercase MGRS reference with five digits per axis")
                    mgrs_indices.add(index)
                else:
                    normalized["mgrs"] = mgrs

        if errors:
            schema_indices.add(index)
        if clock is not None:
            if previous_clock is not None and clock < previous_clock[1]:
                backward_steps += 1
                order_indices.update((previous_clock[0], index))
                errors.append("recorded_at moved backward in acquisition order")
            previous_clock = (index, clock)

        rows.append({"index": index, "valid": not errors, "errors": errors,
                     "sample": None if errors else normalized, "raw": _safe_raw(raw)})

    valid_samples = [row["sample"] for row in rows if row["valid"]]
    valid_count = len(valid_samples)
    metrics = {
        "sample_count": len(samples),
        "valid_count": valid_count,
        "invalid_count": len(samples) - valid_count,
        "timestamp_order_errors": backward_steps,
        "mgrs_count": sum("mgrs" in sample for sample in valid_samples),
        "first_valid_recorded_at": valid_samples[0]["recorded_at"] if valid_samples else None,
        "last_valid_recorded_at": valid_samples[-1]["recorded_at"] if valid_samples else None,
    }
    checks = []

    def check(identifier, status, description, observed, limit, evidence):
        checks.append({"id": identifier, "status": status, "description": description,
                       "observed": observed, "limit": limit, "evidence": sorted(evidence)})

    check("sample_count", "pass" if valid_count >= 2 else "fail", "At least two valid GPS records are required.",
          valid_count, {"minimum_valid_samples": 2}, [row["index"] for row in rows if not row["valid"]])
    check("schema", "fail" if schema_indices else "pass", "Each GPS record must have valid required fields.",
          len(schema_indices), 0, schema_indices)
    check("gps_validity", "fail" if gps_indices else "pass", "Each recorded fix must have valid latitude and longitude.",
          len(gps_indices), 0, gps_indices)
    clock_indices = bad_clock_indices | order_indices
    check("timestamp_order", "fail" if clock_indices else "pass",
          "Recorded times use the fixed +02:00 offset and must be nondecreasing. Equal timestamps are allowed.",
          {"invalid_timestamps": len(bad_clock_indices), "backward_steps": backward_steps},
          {"invalid_timestamps": 0, "backward_steps": 0}, clock_indices)
    mgrs_status = "fail" if mgrs_indices else "pass" if mgrs_present else "skip"
    check("mgrs_format", mgrs_status, "Optional MGRS references use compact uppercase syntax with five digits per axis.",
          len(mgrs_indices), 0, mgrs_indices)
    return {"status": "fail" if any(c["status"] == "fail" for c in checks) else "pass",
            "test_version": GPS_TEST_VERSION, "config": {}, "metrics": metrics, "rows": rows, "checks": checks}


def analyze_samples(samples: list, profile: str, config: dict | None = None) -> dict:
    """Analyze records in acquisition order and return a strict-JSON report.

    Row validity covers field and per-record sequence/timestamp integrity.
    Aggregate checks (including motion) may fail even if records are valid.
    Sequence and timing checks use each parseable field independently, so an
    unrelated bad angle cannot conceal a duplicate packet or sampling gap.
    """
    if not isinstance(samples, list):
        raise ValueError("samples must be a list")
    if len(samples) > 20000:
        raise ValueError("samples must contain at most 20000 records")
    if profile not in TEST_PROFILES:
        raise ValueError("test_profile must be one of " + ", ".join(TEST_PROFILES))
    if profile == "gps_monitoring":
        if config is not None and (not isinstance(config, dict) or config):
            raise ValueError("gps_monitoring requires an empty test_config")
        return _analyze_gps_samples(samples)
    effective = _settings(config)
    rows = []
    schema_indices, angle_indices, gps_indices = set(), set(), set()
    sequence_indices, timestamp_indices, gap_indices = set(), set(), set()
    seen_sequences: dict[int, int] = {}
    previous_sequence = None
    previous_time = None
    missing_count = duplicate_count = backward_count = timestamp_errors = forward_gaps = 0
    intervals = []
    maximum_gap = effective["expected_interval_ms"] * effective["gap_tolerance_factor"]

    for index, raw in enumerate(samples):
        errors: list[str] = []
        normalized = {}
        seq = timestamp = None
        if not isinstance(raw, dict):
            errors.append("sample must be an object")
        else:
            adapter_error = _adapter_failure(raw)
            if adapter_error:
                errors.append(adapter_error)
            value = raw.get("seq")
            if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= MAX_SEQUENCE:
                errors.append("seq must be a nonnegative signed 64-bit integer")
            else:
                seq = value
                normalized["seq"] = seq
            timestamp = _number(raw.get("t_ms"))
            if timestamp is None or timestamp < 0:
                errors.append("t_ms must be a finite nonnegative number")
                timestamp = None
            else:
                normalized["t_ms"] = timestamp
            for field, (low, high) in _ANGLE_BOUNDS.items():
                angle = _number(raw.get(field))
                if angle is None:
                    errors.append(f"{field} must be a finite number")
                    angle_indices.add(index)
                elif not low <= angle <= high:
                    errors.append(f"{field} must be between {low:g} and {high:g} degrees")
                    angle_indices.add(index)
                else:
                    normalized[field] = angle
            if "gps_fix" in raw:
                if not isinstance(raw["gps_fix"], bool):
                    errors.append("gps_fix must be a boolean")
                    gps_indices.add(index)
                else:
                    normalized["gps_fix"] = raw["gps_fix"]
            for field, (low, high) in _GPS_BOUNDS.items():
                if field not in raw:
                    continue
                coordinate = _number(raw[field])
                if coordinate is None or not low <= coordinate <= high:
                    errors.append(f"{field} must be a finite number between {low:g} and {high:g}")
                    gps_indices.add(index)
                else:
                    normalized[field] = coordinate

        if errors:
            schema_indices.add(index)
            normalized = None

        if seq is not None:
            if seq in seen_sequences:
                duplicate_count += 1
                sequence_indices.update((seen_sequences[seq], index))
                errors.append(f"duplicate seq {seq}; first seen at row {seen_sequences[seq]}")
            else:
                seen_sequences[seq] = index
            if previous_sequence is not None:
                previous_index, previous_seq = previous_sequence
                if seq < previous_seq:
                    backward_count += 1
                    sequence_indices.update((previous_index, index))
                    errors.append(f"seq moved backward from {previous_seq} to {seq}")
                elif seq > previous_seq + 1:
                    forward_gaps += 1
                    sequence_indices.update((previous_index, index))
                    errors.append(f"seq jumped from {previous_seq} to {seq} in acquisition order")
            previous_sequence = (index, seq)

        if timestamp is not None:
            if previous_time is not None:
                previous_index, previous_timestamp = previous_time
                interval = timestamp - previous_timestamp
                if interval <= 0:
                    timestamp_errors += 1
                    timestamp_indices.update((previous_index, index))
                    errors.append("t_ms must increase strictly in acquisition order")
                else:
                    intervals.append(interval)
                    if interval > maximum_gap:
                        gap_indices.update((previous_index, index))
                        errors.append(f"sampling gap {interval:g} ms exceeds {maximum_gap:g} ms")
            previous_time = (index, timestamp)

        rows.append({"index": index, "valid": not errors, "errors": errors,
                     "sample": normalized, "raw": _safe_raw(raw)})

    # Count genuinely absent IDs, rather than double-counting a gap that was
    # subsequently backfilled by an out-of-order packet. Ordering still fails.
    if seen_sequences:
        missing_count = max(seen_sequences) - min(seen_sequences) + 1 - len(seen_sequences)
    valid_samples = [row["sample"] for row in rows if row["valid"] and row["sample"] is not None]
    valid_count = len(valid_samples)
    spans = {}
    for field in _ANGLE_BOUNDS:
        values = [sample[field] for sample in valid_samples]
        spans[field] = (max(values) - min(values) if values else 0.0) if field == "pitch_deg" else _circular_span(values)
    motion_observed = max(spans.values())
    checks = []

    def check(identifier, status, description, observed, limit, evidence):
        checks.append({"id": identifier, "status": status, "description": description,
                       "observed": observed, "limit": limit, "evidence": sorted(evidence)})

    check("sample_count", "pass" if valid_count >= 2 else "fail", "At least two valid records are required.",
          valid_count, {"minimum_valid_samples": 2}, [r["index"] for r in rows if not r["valid"]])
    check("schema", "fail" if schema_indices else "pass", "Required fields must have valid types and finite values.",
          len(schema_indices), 0, schema_indices)
    check("sequence_integrity", "fail" if sequence_indices else "pass", "Sequence numbers must be unique, ordered and contiguous.",
          {"duplicates": duplicate_count, "backward_steps": backward_count, "missing_values": missing_count, "forward_gaps": forward_gaps},
          {"duplicates": 0, "backward_steps": 0, "missing_values": 0, "forward_gaps": 0}, sequence_indices)
    check("timestamp_order", "fail" if timestamp_indices else "pass", "Device elapsed time must increase strictly.",
          timestamp_errors, 0, timestamp_indices)
    check("sampling_gaps", "fail" if gap_indices else "pass", "Consecutive timestamps must not exceed the configured maximum gap.",
          {"maximum_interval_ms": max(intervals) if intervals else None}, {"maximum_interval_ms": maximum_gap}, gap_indices)
    check("angle_bounds", "fail" if angle_indices else "pass", "Euler angles must have finite values within the documented supported ranges.",
          len(angle_indices), {name: list(bounds) for name, bounds in _ANGLE_BOUNDS.items()}, angle_indices)
    check("gps_validity", "fail" if gps_indices else "pass", "Optional GPS fields must have valid types and coordinate bounds.",
          len(gps_indices), 0, gps_indices)
    if profile == "monitoring":
        motion_status, description = "skip", "Stationary devices are allowed in monitoring; motion response is not evaluated."
    elif valid_count < 2:
        motion_status, description = "skip", "Insufficient valid records to evaluate guided motion."
    else:
        motion_status = "pass" if motion_observed >= effective["motion_span_deg"] else "fail"
        description = "Guided motion must produce the configured angular span on at least one axis."
    check("motion_response", motion_status, description, {"span_deg": spans},
          {"minimum_span_deg": effective["motion_span_deg"]}, [r["index"] for r in rows if r["valid"]] if motion_status == "fail" else [])

    metrics = {"sample_count": len(samples), "valid_count": valid_count, "invalid_count": len(samples) - valid_count,
               "missing_sequence_count": missing_count, "duplicate_sequence_count": duplicate_count,
               "backward_sequence_count": backward_count, "timestamp_order_errors": timestamp_errors,
               "maximum_interval_ms": max(intervals) if intervals else None, "angle_span_deg": spans}
    return {"status": "fail" if any(c["status"] == "fail" for c in checks) else "pass",
            "test_version": TEST_VERSION, "config": effective, "metrics": metrics, "rows": rows, "checks": checks}
