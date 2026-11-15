"""Immutable, transactional storage for sensor sessions and their test evidence."""

from contextlib import closing
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import sqlite3
import subprocess
import uuid
from zoneinfo import ZoneInfo

from . import __version__
from .quality import TEST_PROFILES, analyze_samples


MAX_SAMPLES = 20_000
REPORT_ID_LENGTH = 12


class InputError(ValueError):
    """Invalid metadata or input structure."""


class SessionConflict(InputError):
    """A session identity has already been used for different input."""


def strict_json(text):
    def reject_constant(value):
        raise InputError(f"Nonstandard JSON number {value} is not allowed")

    def unique_keys(pairs):
        obj = {}
        for key, value in pairs:
            if key in obj:
                raise InputError(f"Duplicate JSON key: {key}")
            obj[key] = value
        return obj

    try:
        return json.loads(text, parse_constant=reject_constant, object_pairs_hook=unique_keys)
    except InputError:
        raise
    except (ValueError, UnicodeDecodeError, RecursionError) as exc:
        raise InputError("Input must be valid UTF-8 JSON") from exc


def canonical_json(value):
    try:
        return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False)
    except (TypeError, ValueError) as exc:
        raise InputError("Input must contain only finite, JSON-compatible values") from exc


def validate_payload(payload):
    if not isinstance(payload, dict):
        raise InputError("Payload must be a JSON object")
    required = {"device_id", "session_id", "firmware_version", "source", "test_profile", "samples"}
    allowed = required | {"scenario", "test_config", "capture"}
    missing = required - payload.keys()
    if missing:
        raise InputError("Missing metadata: " + ", ".join(sorted(missing)))
    extra = payload.keys() - allowed
    if extra:
        raise InputError("Unknown metadata: " + ", ".join(sorted(extra)))
    result = dict(payload)
    for key in ("device_id", "session_id", "firmware_version"):
        value = result[key]
        if not isinstance(value, str) or not value.strip() or len(value) > 120 or not value.isprintable():
            raise InputError(f"{key} must be a nonempty printable string of at most 120 characters")
    if result["source"] not in ("simulator", "hardware"):
        raise InputError("source must be simulator or hardware")
    if result["test_profile"] not in TEST_PROFILES:
        raise InputError("test_profile must be one of " + ", ".join(TEST_PROFILES))
    scenario = result.get("scenario", "")
    if not isinstance(scenario, str) or len(scenario) > 120 or (scenario and not scenario.isprintable()):
        raise InputError("scenario must be a printable string of at most 120 characters")
    samples = result["samples"]
    if not isinstance(samples, list) or not 1 <= len(samples) <= MAX_SAMPLES:
        raise InputError(f"samples must be a nonempty array with at most {MAX_SAMPLES} rows")
    config = result.get("test_config", {})
    if not isinstance(config, dict):
        raise InputError("test_config must be an object")
    result["scenario"] = scenario
    result["test_config"] = config
    if "capture" in result:
        if not isinstance(result["capture"], dict):
            raise InputError("capture provenance must be an object")
        if len(canonical_json(result["capture"]).encode("utf-8")) > 16_384:
            raise InputError("capture provenance must not exceed 16 KiB")
    canonical_json(result)
    return result


def git_revision():
    root = Path(__file__).resolve().parent.parent
    git = shutil.which("git")
    if git is None or not (root / ".git").exists():
        return None
    try:
        result = subprocess.run([git, "rev-parse", "HEAD"], cwd=root, capture_output=True, text=True, timeout=2)
        return result.stdout.strip() if result.returncode == 0 else None
    except (OSError, subprocess.TimeoutExpired):
        return None


SCHEMA = """
CREATE TABLE IF NOT EXISTS devices (
    device_id TEXT PRIMARY KEY,
    first_seen TEXT NOT NULL
);
CREATE TABLE IF NOT EXISTS runs (
    run_id TEXT PRIMARY KEY,
    device_id TEXT NOT NULL REFERENCES devices(device_id),
    session_id TEXT NOT NULL,
    firmware_version TEXT NOT NULL,
    source TEXT NOT NULL CHECK(source IN ('simulator','hardware')),
    test_profile TEXT NOT NULL,
    scenario TEXT NOT NULL,
    created_at TEXT NOT NULL,
    input_hash TEXT NOT NULL,
    test_version TEXT NOT NULL,
    app_version TEXT NOT NULL,
    git_revision TEXT,
    payload_json TEXT NOT NULL,
    analysis_json TEXT NOT NULL,
    UNIQUE(device_id, session_id)
);
CREATE TABLE IF NOT EXISTS samples (
    run_id TEXT NOT NULL REFERENCES runs(run_id),
    row_index INTEGER NOT NULL,
    valid INTEGER NOT NULL CHECK(valid IN (0,1)),
    seq INTEGER,
    t_ms REAL,
    roll_deg REAL,
    pitch_deg REAL,
    yaw_deg REAL,
    errors_json TEXT NOT NULL,
    raw_json TEXT NOT NULL,
    PRIMARY KEY(run_id, row_index)
);
CREATE TABLE IF NOT EXISTS checks (
    run_id TEXT NOT NULL REFERENCES runs(run_id),
    check_id TEXT NOT NULL,
    status TEXT NOT NULL CHECK(status IN ('pass','fail','skip')),
    description TEXT NOT NULL,
    observed_json TEXT NOT NULL,
    limit_json TEXT NOT NULL,
    evidence_json TEXT NOT NULL,
    PRIMARY KEY(run_id, check_id)
);
CREATE INDEX IF NOT EXISTS runs_created ON runs(created_at);
CREATE INDEX IF NOT EXISTS checks_status ON checks(status);
"""


class Store:
    def __init__(self, path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with closing(self.connect()) as conn:
            conn.executescript(SCHEMA)
            conn.commit()

    def connect(self):
        conn = sqlite3.connect(self.path, timeout=5)
        conn.row_factory = sqlite3.Row
        conn.execute("PRAGMA foreign_keys = ON")
        conn.execute("PRAGMA journal_mode = WAL")
        return conn

    def ingest(self, payload):
        # Keep the submitted input independent of default test settings. Effective
        # configuration is stored in analysis, so later default changes cannot
        # rewrite provenance or change the identity of an identical replay.
        submitted = canonical_json(payload)
        payload = validate_payload(payload)
        try:
            analysis = analyze_samples(payload["samples"], payload["test_profile"], payload["test_config"])
        except ValueError as exc:
            raise InputError(str(exc)) from exc
        raw = submitted
        digest = hashlib.sha256(raw.encode("utf-8")).hexdigest()
        created_at = datetime.now(timezone.utc).isoformat(timespec="seconds")
        with closing(self.connect()) as conn, conn:
            conn.execute("BEGIN IMMEDIATE")
            existing = conn.execute("SELECT run_id,input_hash FROM runs WHERE device_id=? AND session_id=?",
                                    (payload["device_id"], payload["session_id"])).fetchone()
            if existing:
                if existing["input_hash"] != digest:
                    raise SessionConflict("This device/session identity already has different input; use a new session_id")
                run_id, created = existing["run_id"], False
            else:
                run_id, created = uuid.uuid4().hex, True
                conn.execute("INSERT OR IGNORE INTO devices VALUES (?,?)", (payload["device_id"], created_at))
                conn.execute("INSERT INTO runs VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?)", (
                    run_id, payload["device_id"], payload["session_id"], payload["firmware_version"],
                    payload["source"], payload["test_profile"], payload["scenario"], created_at, digest,
                    analysis["test_version"], __version__, git_revision(), raw, canonical_json(analysis)))
                for row in analysis["rows"]:
                    sample = row["sample"] or {}
                    conn.execute("INSERT INTO samples VALUES (?,?,?,?,?,?,?,?,?,?)", (
                        run_id, row["index"], int(row["valid"]), sample.get("seq"), sample.get("t_ms"),
                        sample.get("roll_deg"), sample.get("pitch_deg"), sample.get("yaw_deg"),
                        canonical_json(row["errors"]), canonical_json(row["raw"])))
                for check in analysis["checks"]:
                    conn.execute("INSERT INTO checks VALUES (?,?,?,?,?,?,?)", (
                        run_id, check["id"], check["status"], check["description"],
                        canonical_json(check["observed"]), canonical_json(check["limit"]),
                        canonical_json(check["evidence"])))
        return {"run": self.get_run(run_id)["run"], "created": created}

    @staticmethod
    def summary(row):
        analysis = json.loads(row["analysis_json"])
        keys = ("run_id", "device_id", "session_id", "firmware_version", "source", "test_profile",
                "scenario", "created_at", "input_hash", "test_version", "app_version", "git_revision")
        result = {key: row[key] for key in keys}
        submitted = json.loads(row["payload_json"])
        if "capture" in submitted:
            result["capture"] = submitted["capture"]
        result.update({key: analysis["metrics"][key] for key in ("sample_count", "valid_count", "invalid_count")})
        result["status"] = analysis["status"]
        for status, label in (("pass", "passed_checks"), ("fail", "failed_checks"), ("skip", "skipped_checks")):
            result[label] = sum(c["status"] == status for c in analysis["checks"])
        return result

    def list_runs(self, limit=100):
        if not isinstance(limit, int) or isinstance(limit, bool) or not 1 <= limit <= 500:
            raise InputError("limit must be an integer between 1 and 500")
        with closing(self.connect()) as conn:
            rows = conn.execute("SELECT * FROM runs ORDER BY created_at DESC, rowid DESC LIMIT ?", (limit,)).fetchall()
        return [self.summary(row) for row in rows]

    def get_run(self, run_id):
        with closing(self.connect()) as conn:
            row = conn.execute("SELECT * FROM runs WHERE run_id=?", (run_id,)).fetchone()
        if row is None:
            return None
        return {"run": self.summary(row), "analysis": json.loads(row["analysis_json"])}

    def markdown_report(self, run_id):
        detail = self.get_run(run_id)
        if detail is None:
            return None
        run, analysis = detail["run"], detail["analysis"]
        created_at = datetime.fromisoformat(run["created_at"]).astimezone(ZoneInfo("Europe/Helsinki"))
        utc_offset = created_at.isoformat(timespec="seconds")[-6:]
        report_values = {
            **run,
            "run_id": run["run_id"][:REPORT_ID_LENGTH],
            "created_at": f"{created_at.day}.{created_at.month}.{created_at:%Y} klo {created_at:%H.%M.%S} (UTC{utc_offset})",
        }
        lines = ["# SensorBench test report", "", f"Overall result: **{run['status'].upper()}**", ""]
        for key in ("run_id", "device_id", "session_id", "source", "scenario", "test_profile", "firmware_version",
                    "created_at", "input_hash", "test_version", "app_version", "git_revision"):
            value = report_values[key]
            lines.append(f"- {key}: {value if value is not None else 'not available'}")
        lines += ["", "## Effective test configuration", "", "```json", json.dumps(analysis["config"], indent=2), "```",
                  "", "## Metrics", "", "```json", json.dumps(analysis["metrics"], indent=2), "```", "", "## Checks", ""]
        if "capture" in run:
            lines[lines.index("## Checks"):lines.index("## Checks")] = [
                "## Capture provenance", "", "```json", json.dumps(run["capture"], ensure_ascii=False, indent=2), "```", ""]
        for check in analysis["checks"]:
            lines += [f"### {check['id']} {check['status'].upper()}", "", check["description"],
                      f"- Observed: {json.dumps(check['observed'], ensure_ascii=False)}",
                      f"- Limit: {json.dumps(check['limit'], ensure_ascii=False)}",
                      f"- Evidence input row indices: {check['evidence']}", ""]
        lines += ["## Interpretation", ""]
        if run["test_profile"] == "gps_monitoring":
            lines += ["GPS monitoring checks recorded fixes, clock order and optional MGRS syntax.",
                      "The recorded clock uses the firmware's fixed +02:00 offset. Equal readings are allowed.",
                      "Raw sample evidence retains the submitted coordinates and any original lines.", ""]
        else:
            lines += ["Simulator runs use synthetic data and known injected faults. Hardware imports are labelled separately.",
                      "Guided motion checks assume the device was deliberately moved. A pass is not a calibrated accuracy measurement.",
                      "This increment contains deterministic checks; it does not use an AI agent or a machine learning model.", ""]
        return "\n".join(lines)
