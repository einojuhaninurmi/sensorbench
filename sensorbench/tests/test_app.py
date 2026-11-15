from contextlib import closing
import copy
import http.client
import json
from pathlib import Path
import sqlite3
import tempfile
import threading
import unittest
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from sensorbench.server import MAX_BODY, make_server
from sensorbench.simulator import generate_samples
from sensorbench.storage import InputError, SessionConflict, Store, strict_json


def payload(session="test-normal", scenario="normal"):
    return {"device_id": "sim-test-01", "session_id": session, "firmware_version": "test-0.1",
            "source": "simulator", "test_profile": "guided_motion", "scenario": scenario,
            "samples": generate_samples(scenario)}


class StorageTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.store = Store(Path(self.temp.name) / "test.sqlite3")

    def test_identical_replay_is_idempotent_and_preserves_samples(self):
        first = self.store.ingest(payload())
        second = self.store.ingest(payload())
        self.assertTrue(first["created"])
        self.assertFalse(second["created"])
        self.assertEqual(first["run"]["run_id"], second["run"]["run_id"])
        with closing(self.store.connect()) as conn:
            self.assertEqual(conn.execute("SELECT COUNT(*) FROM runs").fetchone()[0], 1)
            self.assertEqual(conn.execute("SELECT COUNT(*) FROM samples").fetchone()[0], 200)

    def test_conflicting_session_does_not_change_original(self):
        original = payload()
        first = self.store.ingest(original)
        changed = copy.deepcopy(original)
        changed["samples"][0]["roll_deg"] = 99
        with self.assertRaises(SessionConflict):
            self.store.ingest(changed)
        detail = self.store.get_run(first["run"]["run_id"])
        self.assertEqual(detail["analysis"]["rows"][0]["raw"], original["samples"][0])

    def test_bad_sample_is_retained_and_failed_not_silently_dropped(self):
        data = payload()
        data["samples"][5] = {"seq": 5, "t_ms": "bad clock"}
        run = self.store.ingest(data)["run"]
        self.assertEqual(run["status"], "fail")
        self.assertEqual(run["invalid_count"], 1)
        detail = self.store.get_run(run["run_id"])
        row = detail["analysis"]["rows"][5]
        self.assertFalse(row["valid"])
        self.assertEqual(row["raw"], data["samples"][5])
        self.assertTrue(row["errors"])

    def test_config_errors_leave_no_partial_run(self):
        data = payload()
        data["test_config"] = {"expected_interval_ms": -1}
        with self.assertRaises(InputError):
            self.store.ingest(data)
        self.assertEqual(self.store.list_runs(), [])
        with closing(self.store.connect()) as conn:
            self.assertEqual(conn.execute("SELECT COUNT(*) FROM samples").fetchone()[0], 0)

    def test_run_has_provenance_and_report(self):
        run = self.store.ingest(payload())["run"]
        self.assertEqual(len(run["input_hash"]), 64)
        self.assertTrue(run["test_version"])
        self.assertTrue(run["app_version"])
        report = self.store.markdown_report(run["run_id"])
        self.assertIn(f'- run_id: {run["run_id"][:12]}', report.splitlines())
        self.assertNotIn(run["run_id"], report)
        for text in (run["input_hash"], "simulator", "Effective test configuration", "Evidence input row indices"):
            self.assertIn(text, report)

    def test_report_formats_existing_utc_timestamps_in_finnish_time(self):
        run_id = self.store.ingest(payload())["run"]["run_id"]
        cases = [
            ("2026-11-07T20:55:03+00:00", "7.11.2026 klo 22.55.03 (UTC+02:00)"),
            ("2026-07-07T22:55:03+00:00", "8.7.2026 klo 01.55.03 (UTC+03:00)"),
        ]
        for stored, expected in cases:
            with self.subTest(stored=stored):
                with closing(self.store.connect()) as conn, conn:
                    conn.execute("UPDATE runs SET created_at=? WHERE run_id=?", (stored, run_id))
                report = self.store.markdown_report(run_id)
                self.assertIn(f"- created_at: {expected}", report.splitlines())
                self.assertEqual(self.store.get_run(run_id)["run"]["created_at"], stored)

    def test_distinct_sessions_are_not_deduplicated(self):
        first = self.store.ingest(payload("session-one"))
        second = self.store.ingest(payload("session-two"))
        self.assertNotEqual(first["run"]["run_id"], second["run"]["run_id"])
        self.assertEqual(len(self.store.list_runs()), 2)

    def test_original_config_and_replay_survive_default_changes(self):
        data = payload()
        data["test_config"] = {"expected_interval_ms": 20}
        first = self.store.ingest(data)
        with closing(self.store.connect()) as conn:
            stored = json.loads(conn.execute("SELECT payload_json FROM runs").fetchone()[0])
        self.assertEqual(stored, data)
        self.assertNotIn("gap_tolerance_factor", stored["test_config"])
        with patch.dict("sensorbench.quality.DEFAULT_CONFIG", {"gap_tolerance_factor": 5.0}):
            replay = self.store.ingest(data)
        self.assertFalse(replay["created"])
        self.assertEqual(replay["run"]["run_id"], first["run"]["run_id"])
        detail = self.store.get_run(first["run"]["run_id"])
        self.assertEqual(detail["analysis"]["config"]["gap_tolerance_factor"], 3.0)

    def test_failed_transaction_rolls_back_all_evidence_tables(self):
        with closing(self.store.connect()) as conn, conn:
            conn.execute("CREATE TRIGGER force_check_failure BEFORE INSERT ON checks BEGIN SELECT RAISE(ABORT, 'test-only failure'); END")
        with self.assertRaises(sqlite3.IntegrityError):
            self.store.ingest(payload())
        with closing(self.store.connect()) as conn:
            for table in ("devices", "runs", "samples", "checks"):
                self.assertEqual(conn.execute("SELECT COUNT(*) FROM " + table).fetchone()[0], 0, table)

    def test_metadata_and_strict_json_rejection(self):
        for data in ({}, {**payload(), "samples": []}, {**payload(), "source": "live-aircraft"},
                     {**payload(), "device_id": ""}, {**payload(), "extra": True}):
            with self.subTest(metadata=list(data)):
                with self.assertRaises(InputError):
                    self.store.ingest(data)
        for raw in ('{"a":NaN}', '{"a":Infinity}', '{"a":1,"a":2}', '{broken'):
            with self.assertRaises(InputError):
                strict_json(raw)

    def test_capture_provenance_is_preserved_exposed_and_reported(self):
        data = payload()
        data["capture"] = {"transport": "recorded-ndjson", "protocol": "sensorbench-quaternion-v1",
                           "raw_log_sha256": "a" * 64, "adapter_version": "test"}
        run = self.store.ingest(data)["run"]
        self.assertEqual(run["capture"], data["capture"])
        self.assertEqual(self.store.get_run(run["run_id"])["run"]["capture"], data["capture"])
        report = self.store.markdown_report(run["run_id"])
        self.assertIn("## Capture provenance", report)
        self.assertIn("a" * 64, report)
        for bad in ([], {"large": "x" * 16385}):
            with self.assertRaises(InputError):
                self.store.ingest({**payload("bad-capture"), "capture": bad})


class APITests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.server = make_server(Path(cls.temp.name) / "api.sqlite3", port=0)
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()
        cls.base = f"http://127.0.0.1:{cls.server.server_port}"

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()
        cls.thread.join(timeout=3)
        cls.temp.cleanup()

    def request(self, path, data=None, headers=None, raw=None):
        body = raw if raw is not None else (json.dumps(data).encode() if data is not None else None)
        merged = {"Content-Type": "application/json", **(headers or {})}
        request = Request(self.base + path, data=body, headers=merged)
        try:
            response = urlopen(request, timeout=5)
        except HTTPError as exc:
            response = exc
        with response:
            return response.status, response.read(), response.headers

    def test_health_and_dashboard(self):
        status, body, _ = self.request("/api/health")
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(body)["status"], "ok")
        status, body, headers = self.request("/")
        self.assertEqual(status, 200)
        self.assertIn(b"SensorBench", body)
        self.assertEqual(headers["X-Content-Type-Options"], "nosniff")

    def test_end_to_end_ingest_fetch_replay_and_report(self):
        data = payload("api-complete")
        status, body, _ = self.request("/api/runs", data)
        self.assertEqual(status, 201)
        run_id = json.loads(body)["run"]["run_id"]
        status, body, _ = self.request("/api/runs/" + run_id)
        self.assertEqual(status, 200)
        detail = json.loads(body)
        self.assertEqual(detail["run"]["status"], "pass")
        self.assertEqual(detail["analysis"]["rows"][0]["raw"], data["samples"][0])
        self.assertEqual(self.request("/api/runs", data)[0], 200)
        changed = copy.deepcopy(data)
        changed["firmware_version"] = "changed"
        self.assertEqual(self.request("/api/runs", changed)[0], 409)
        status, body, headers = self.request("/api/runs/" + run_id + "/report")
        self.assertEqual(status, 200)
        self.assertIn(f"- run_id: {run_id[:12]}".encode(), body.splitlines())
        self.assertEqual(headers["Content-Disposition"], f'attachment; filename="sensorbench-{run_id[:12]}.md"')

    def test_failed_fault_run_is_visible_in_list(self):
        status, body, _ = self.request("/api/runs", payload("api-frozen", "frozen"))
        self.assertEqual(status, 201)
        self.assertEqual(json.loads(body)["run"]["status"], "fail")
        status, body, _ = self.request("/api/runs")
        self.assertEqual(status, 200)
        self.assertTrue(any(r["session_id"] == "api-frozen" and r["failed_checks"] > 0 for r in json.loads(body)["runs"]))

    def test_invalid_input_has_clear_errors(self):
        self.assertEqual(self.request("/api/runs", raw=b'{"samples":NaN}')[0], 400)
        self.assertEqual(self.request("/api/runs", raw=b"{broken")[0], 400)
        self.assertEqual(self.request("/api/runs", raw=b"{}", headers={"Content-Type": "text/plain"})[0], 415)
        self.assertEqual(self.request("/api/runs?limit=0")[0], 400)
        self.assertEqual(self.request("/api/runs/not-present")[0], 404)
        self.assertEqual(self.request("/not-a-route")[0], 404)

    def test_cross_origin_ingestion_is_rejected(self):
        self.assertEqual(self.request("/api/runs", payload("bad-origin"), headers={"Origin": "https://example.com"})[0], 403)

    def test_oversized_sequence_is_a_traceable_failed_measurement(self):
        data = payload("api-large-sequence")
        data["samples"][8]["seq"] = 2 ** 63
        status, body, _ = self.request("/api/runs", data)
        self.assertEqual(status, 201)
        run = json.loads(body)["run"]
        self.assertEqual(run["status"], "fail")
        status, body, _ = self.request("/api/runs/" + run["run_id"])
        self.assertEqual(status, 200)
        row = json.loads(body)["analysis"]["rows"][8]
        self.assertFalse(row["valid"])
        self.assertEqual(row["raw"]["seq"], 2 ** 63)

    def test_body_limit_checked_before_reading(self):
        connection = http.client.HTTPConnection("127.0.0.1", self.server.server_port, timeout=5)
        try:
            connection.putrequest("POST", "/api/runs")
            connection.putheader("Content-Type", "application/json")
            connection.putheader("Content-Length", str(MAX_BODY + 1))
            connection.endheaders()
            response = connection.getresponse()
            self.assertEqual(response.status, 413)
            response.read()
        finally:
            connection.close()


if __name__ == "__main__":
    unittest.main()
