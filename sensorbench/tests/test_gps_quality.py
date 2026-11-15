"""GPS fix evidence passes through the quality engine and SQLite storage."""

from contextlib import closing
import json
import math
import sqlite3
import unittest
from unittest.mock import patch

from sensorbench.quality import GPS_TEST_VERSION, analyze_samples
from sensorbench.storage import Store


def fix(second="00", **changes):
    return {"recorded_at": f"2026-11-07T10:25:{second}+02:00", "gps_fix": True,
            "latitude_deg": 60.17, "longitude_deg": 24.94,
            "original_line": f"07.11.2026 10:25:{second} 60.17 24.94", **changes}


def checks(result):
    return {check["id"]: check for check in result["checks"]}


class GPSQualityTests(unittest.TestCase):
    def test_equal_timestamps_and_optional_mgrs_pass_without_imu_fields(self):
        samples = [fix(mgrs="35VLG1234512345"), fix()]
        result = analyze_samples(samples, "gps_monitoring")
        self.assertEqual(result["status"], "pass")
        self.assertEqual(result["test_version"], GPS_TEST_VERSION)
        self.assertEqual(result["metrics"]["valid_count"], 2)
        self.assertEqual(result["metrics"]["mgrs_count"], 1)
        self.assertEqual(checks(result)["timestamp_order"]["status"], "pass")
        self.assertEqual(result["rows"][0]["raw"], samples[0])
        self.assertEqual(result["rows"][0]["sample"], {key: value for key, value in samples[0].items() if key != "original_line"})
        self.assertNotIn("mgrs", result["rows"][1]["sample"])
        self.assertNotIn("motion_response", checks(result))
        json.dumps(result, allow_nan=False)

    def test_absent_mgrs_is_skipped_and_at_least_two_fixes_are_required(self):
        for count in (0, 1, 2):
            with self.subTest(count=count):
                result = analyze_samples([fix()] * count, "gps_monitoring")
                self.assertEqual(result["status"], "pass" if count == 2 else "fail")
                self.assertEqual(checks(result)["mgrs_format"]["status"], "skip")

    def test_backward_clock_is_failed_in_original_order(self):
        samples = [fix("10"), fix("09"), fix("11")]
        result = analyze_samples(samples, "gps_monitoring")
        self.assertEqual(result["status"], "fail")
        self.assertEqual(result["metrics"]["timestamp_order_errors"], 1)
        self.assertEqual(checks(result)["timestamp_order"]["evidence"], [0, 1])
        self.assertEqual([row["raw"] for row in result["rows"]], samples)
        self.assertFalse(result["rows"][1]["valid"])
        self.assertIsNone(result["rows"][1]["sample"])

    def test_clock_needs_a_real_date_time_and_the_recorded_offset(self):
        invalid = (None, "2026-02-30T10:25:00+02:00", "2026-11-07T10:25:00",
                   "2026-11-07T10:25:00Z", "2026-11-07T10:25:00+03:00")
        for value in invalid:
            with self.subTest(value=value):
                result = analyze_samples([fix(), fix(recorded_at=value)], "gps_monitoring")
                self.assertEqual(result["status"], "fail")
                self.assertEqual(checks(result)["timestamp_order"]["evidence"], [1])

    def test_fix_and_coordinate_pair_are_required(self):
        bad_samples = [fix(gps_fix=False), fix(gps_fix=1), fix(latitude_deg=90.1),
                       fix(longitude_deg=math.nan), fix(latitude_deg="60.17")]
        without_longitude = fix()
        del without_longitude["longitude_deg"]
        bad_samples.append(without_longitude)
        for sample in bad_samples:
            with self.subTest(sample=sample):
                result = analyze_samples([fix(), sample], "gps_monitoring")
                self.assertEqual(result["status"], "fail")
                self.assertEqual(checks(result)["gps_validity"]["evidence"], [1])
                self.assertIsNone(result["rows"][1]["sample"])
                json.dumps(result, allow_nan=False)

    def test_mgrs_syntax_guards_zone_letters_and_precision(self):
        for reference in ("1CDE1234512345", "01CDE1234512345", "60XWV1234512345", "ZAB1234512345"):
            with self.subTest(reference=reference):
                self.assertEqual(analyze_samples([fix(), fix(mgrs=reference)], "gps_monitoring")["status"], "pass")
        for reference in ("00VLG1234512345", "61VLG1234512345", "35VLI1234512345", "35vlg1234512345",
                          "35VLG 12345 12345", "35VLG123451234", None):
            with self.subTest(reference=reference):
                result = analyze_samples([fix(), fix(mgrs=reference)], "gps_monitoring")
                self.assertEqual(result["status"], "fail")
                self.assertEqual(checks(result)["mgrs_format"]["evidence"], [1])

    def test_adapter_failure_retains_original_input(self):
        raw = {"original_line": "broken clock and coordinates", "adapter_error": "Bad date"}
        result = analyze_samples([fix(), raw], "gps_monitoring")
        self.assertEqual(result["status"], "fail")
        self.assertEqual(result["rows"][1]["raw"], raw)
        self.assertIn("telemetry adapter: Bad date", result["rows"][1]["errors"])

    def test_gps_profile_rejects_unused_imu_thresholds(self):
        with self.assertRaises(ValueError):
            analyze_samples([fix(), fix()], "gps_monitoring", {"expected_interval_ms": 20})

    def test_sql_ingest_replay_and_report_preserve_gps_evidence(self):
        uri = "file:sensorbench-gps-test?mode=memory&cache=shared"

        def connect():
            connection = sqlite3.connect(uri, uri=True)
            connection.row_factory = sqlite3.Row
            connection.execute("PRAGMA foreign_keys = ON")
            return connection

        with closing(connect()), patch.object(Store, "connect", side_effect=connect):
            store = Store(":memory:")
            payload = {"device_id": "gps-test", "session_id": "test-fixes", "firmware_version": "test-0.1",
                       "source": "simulator", "test_profile": "gps_monitoring", "samples": [fix(), fix("01")]}
            inserted = store.ingest(payload)
            self.assertTrue(inserted["created"])
            self.assertEqual(inserted["run"]["status"], "pass")
            run_id = inserted["run"]["run_id"]
            self.assertFalse(store.ingest(payload)["created"])
            self.assertEqual(store.get_run(run_id)["analysis"]["rows"][0]["raw"], payload["samples"][0])
            with closing(connect()) as connection:
                row = connection.execute("SELECT seq, t_ms, roll_deg, pitch_deg, yaw_deg, raw_json FROM samples").fetchone()
            self.assertEqual(tuple(row)[:5], (None,) * 5)
            self.assertEqual(json.loads(row["raw_json"]), payload["samples"][0])
            report = store.markdown_report(run_id)
            self.assertIn("GPS monitoring checks recorded fixes", report)
            self.assertIn("+02:00", report)
            self.assertNotIn("Guided motion checks", report)


if __name__ == "__main__":
    unittest.main()
