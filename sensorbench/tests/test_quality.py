"""Quality checks exercise real integrity boundaries and simulated known faults."""

import copy
import json
import math
import unittest

from sensorbench.quality import DEFAULT_CONFIG, analyze_samples
from sensorbench.simulator import SCENARIOS, generate_samples


def checks_by_id(result):
    return {check["id"]: check for check in result["checks"]}


class QualityTests(unittest.TestCase):
    def normal(self, count=40):
        return generate_samples("normal", count=count)

    def test_normal_guided_motion_passes_and_report_contract(self):
        samples = self.normal()
        before = copy.deepcopy(samples)
        result = analyze_samples(samples, "guided_motion")
        self.assertEqual(result["status"], "pass")
        self.assertEqual(result["metrics"]["valid_count"], 40)
        self.assertEqual(result["metrics"]["invalid_count"], 0)
        self.assertEqual(samples, before)
        self.assertEqual(result["rows"][7]["raw"], samples[7])
        self.assertEqual(result["rows"][7]["index"], 7)
        self.assertTrue(result["test_version"])
        json.dumps(result, allow_nan=False)
        for check in result["checks"]:
            self.assertEqual(set(check), {"id", "status", "description", "observed", "limit", "evidence"})

    def test_empty_and_single_record_cannot_pass(self):
        for samples in ([], self.normal()[:1]):
            with self.subTest(count=len(samples)):
                self.assertEqual(analyze_samples(samples, "monitoring")["status"], "fail")

    def test_two_valid_records_can_pass_if_motion_is_exercised(self):
        samples = self.normal()[:2]
        samples[1]["roll_deg"] = samples[0]["roll_deg"] + 3
        self.assertEqual(analyze_samples(samples, "guided_motion")["status"], "pass")

    def test_stationarity_is_not_a_monitoring_fault(self):
        samples = generate_samples("frozen", count=20)
        monitoring = analyze_samples(samples, "monitoring")
        guided = analyze_samples(samples, "guided_motion")
        self.assertEqual(monitoring["status"], "pass")
        self.assertEqual(checks_by_id(monitoring)["motion_response"]["status"], "skip")
        self.assertEqual(guided["status"], "fail")
        self.assertEqual(checks_by_id(guided)["motion_response"]["status"], "fail")
        self.assertEqual(guided["metrics"]["valid_count"], 20)

    def test_packet_loss_preserves_gap_evidence(self):
        samples = generate_samples("packet_loss", count=40)
        self.assertEqual(len(samples), 36)
        result = analyze_samples(samples, "guided_motion")
        self.assertEqual(result["status"], "fail")
        self.assertEqual(result["metrics"]["missing_sequence_count"], 4)
        checks = checks_by_id(result)
        self.assertEqual(checks["sequence_integrity"]["status"], "fail")
        self.assertEqual(checks["sampling_gaps"]["status"], "fail")
        self.assertEqual(checks["sampling_gaps"]["observed"]["maximum_interval_ms"], 100)
        self.assertEqual(checks["sampling_gaps"]["evidence"], [19, 20])
        self.assertEqual(samples[20]["seq"], 24)

    def test_out_of_range_angle_fails_without_concealing_other_samples(self):
        result = analyze_samples(generate_samples("out_of_range", count=20), "guided_motion")
        self.assertEqual(result["status"], "fail")
        self.assertEqual(checks_by_id(result)["angle_bounds"]["evidence"], [10])
        self.assertFalse(result["rows"][10]["valid"])
        self.assertIsNone(result["rows"][10]["sample"])
        self.assertEqual(result["rows"][10]["raw"]["pitch_deg"], 120)

    def test_duplicate_and_backward_sequence_are_not_repaired(self):
        samples = self.normal()
        samples[4]["seq"] = 3
        samples[9]["seq"] = 2
        result = analyze_samples(samples, "monitoring")
        self.assertEqual(result["status"], "fail")
        self.assertEqual(result["metrics"]["duplicate_sequence_count"], 2)
        self.assertEqual(result["metrics"]["backward_sequence_count"], 1)
        self.assertEqual(result["rows"][9]["sample"]["seq"], 2)
        self.assertFalse(result["rows"][9]["valid"])

    def test_duplicate_and_reversed_timestamps_fail(self):
        for timestamp in (40, 10):
            samples = self.normal()
            samples[3]["t_ms"] = timestamp
            result = analyze_samples(samples, "monitoring")
            self.assertEqual(checks_by_id(result)["timestamp_order"]["status"], "fail")
            self.assertIn(3, checks_by_id(result)["timestamp_order"]["evidence"])

    def test_backfilled_out_of_order_packets_are_not_counted_as_lost(self):
        samples = self.normal()[:4]
        samples[1]["seq"], samples[2]["seq"] = 2, 1
        result = analyze_samples(samples, "monitoring")
        self.assertEqual(result["status"], "fail")
        self.assertEqual(result["metrics"]["missing_sequence_count"], 0)
        self.assertEqual(result["metrics"]["backward_sequence_count"], 1)
        self.assertEqual([row["sample"]["seq"] for row in result["rows"]], [0, 2, 1, 3])

    def test_sampling_gap_boundary_is_inclusive(self):
        samples = self.normal()[:2]
        samples[1]["t_ms"] = 60
        self.assertEqual(checks_by_id(analyze_samples(samples, "monitoring"))["sampling_gaps"]["status"], "pass")
        samples[1]["t_ms"] = 60.001
        self.assertEqual(checks_by_id(analyze_samples(samples, "monitoring"))["sampling_gaps"]["status"], "fail")

    def test_malformed_records_and_nonfinite_fields_are_reportable(self):
        bad_values = [None, [], "bad", {"seq": 0}, {**self.normal()[0], "roll_deg": math.nan},
                      {**self.normal()[0], "pitch_deg": math.inf}, {**self.normal()[0], "t_ms": -1},
                      {**self.normal()[0], "seq": True}, {**self.normal()[0], "yaw_deg": "12"}]
        for bad in bad_values:
            with self.subTest(value=bad):
                result = analyze_samples([bad] + self.normal()[1:4], "monitoring")
                self.assertEqual(result["status"], "fail")
                self.assertFalse(result["rows"][0]["valid"])
                self.assertTrue(result["rows"][0]["errors"])
                json.dumps(result, allow_nan=False)

    def test_optional_gps_fields_are_validated_if_present(self):
        for field, value in [("gps_fix", 1), ("latitude_deg", 90.1), ("longitude_deg", -180.1), ("latitude_deg", math.nan)]:
            samples = self.normal()
            samples[5][field] = value
            result = analyze_samples(samples, "monitoring")
            self.assertEqual(checks_by_id(result)["gps_validity"]["status"], "fail")
            self.assertIn(5, checks_by_id(result)["gps_validity"]["evidence"])
        samples = self.normal()
        for sample in samples:
            for field in ("gps_fix", "latitude_deg", "longitude_deg"):
                sample.pop(field)
        self.assertEqual(analyze_samples(samples, "monitoring")["status"], "pass")

    def test_oversized_sequence_is_invalid_with_raw_evidence_preserved(self):
        samples = self.normal()
        samples[0]["seq"] = 1 << 63
        result = analyze_samples(samples, "monitoring")
        self.assertEqual(result["status"], "fail")
        self.assertFalse(result["rows"][0]["valid"])
        self.assertIsNone(result["rows"][0]["sample"])
        self.assertEqual(result["rows"][0]["raw"]["seq"], 1 << 63)
        self.assertIn("64-bit", " ".join(result["rows"][0]["errors"]))
        json.dumps(result, allow_nan=False)
        boundary = self.normal()[:2]
        boundary[0]["seq"], boundary[1]["seq"] = (1 << 63) - 2, (1 << 63) - 1
        self.assertEqual(analyze_samples(boundary, "monitoring")["status"], "pass")

    def test_adapter_failure_invalidates_otherwise_valid_angles_and_preserves_reason(self):
        samples = self.normal()
        samples[3]["adapter_error"] = "Quaternion norm was zero"
        result = analyze_samples(samples, "monitoring")
        self.assertEqual(result["status"], "fail")
        self.assertEqual(result["test_version"], "sensor-quality-0.2")
        row = result["rows"][3]
        self.assertFalse(row["valid"])
        self.assertIsNone(row["sample"])
        self.assertEqual(row["raw"], samples[3])
        self.assertIn("telemetry adapter: Quaternion norm was zero", row["errors"])
        self.assertIn(3, checks_by_id(result)["schema"]["evidence"])

    def test_malformed_adapter_error_always_fails_the_record(self):
        for value in (None, True, 0, [], {}, "", "   ", "x" * 1001):
            with self.subTest(value=value):
                samples = self.normal()
                samples[3]["adapter_error"] = value
                result = analyze_samples(samples, "monitoring")
                row = result["rows"][3]
                self.assertEqual(result["status"], "fail")
                self.assertFalse(row["valid"])
                self.assertIsNone(row["sample"])
                self.assertEqual(row["raw"]["adapter_error"], value)
                self.assertTrue(any("adapter_error must be" in error for error in row["errors"]))

    def test_huge_numeric_fields_are_invalid_instead_of_overflowing(self):
        huge = 10 ** 1000
        for field in ("t_ms", "roll_deg", "latitude_deg"):
            with self.subTest(field=field):
                samples = self.normal()
                samples[0][field] = huge
                result = analyze_samples(samples, "monitoring")
                self.assertEqual(result["status"], "fail")
                self.assertFalse(result["rows"][0]["valid"])
                self.assertIsNone(result["rows"][0]["sample"])
                self.assertEqual(result["rows"][0]["raw"][field], huge)
                json.dumps(result, allow_nan=False)
        with self.assertRaises(ValueError):
            analyze_samples(self.normal(), "monitoring", {"expected_interval_ms": huge})

    def test_wraparound_does_not_create_false_large_motion(self):
        samples = self.normal()[:2]
        for sample, yaw, roll in zip(samples, (359.8, 0.2), (179.8, -179.8)):
            sample.update(roll_deg=roll, pitch_deg=0, yaw_deg=yaw)
        result = analyze_samples(samples, "guided_motion")
        self.assertEqual(result["status"], "fail")
        self.assertAlmostEqual(result["metrics"]["angle_span_deg"]["yaw_deg"], 0.4)
        self.assertAlmostEqual(result["metrics"]["angle_span_deg"]["roll_deg"], 0.4)

    def test_motion_threshold_and_config_do_not_mutate_defaults(self):
        samples = self.normal()[:2]
        for sample, roll in zip(samples, (0, 2)):
            sample.update(roll_deg=roll, pitch_deg=0, yaw_deg=0)
        self.assertEqual(analyze_samples(samples, "guided_motion")["status"], "pass")
        config = {"motion_span_deg": 2.1}
        self.assertEqual(analyze_samples(samples, "guided_motion", config)["status"], "fail")
        self.assertEqual(config, {"motion_span_deg": 2.1})
        self.assertEqual(DEFAULT_CONFIG["motion_span_deg"], 2)

    def test_bad_profile_config_and_top_level_raise(self):
        with self.assertRaises(ValueError):
            analyze_samples([], "unknown")
        with self.assertRaises(ValueError):
            analyze_samples({}, "monitoring")
        for config in ([], {"typo": 1}, {"expected_interval_ms": 0}, {"expected_interval_ms": math.inf},
                       {"gap_tolerance_factor": 0.9}, {"motion_span_deg": -1}, {"motion_span_deg": 361},
                       {"motion_span_deg": True}, {"expected_interval_ms": 1e308, "gap_tolerance_factor": 3}):
            with self.subTest(config=config), self.assertRaises(ValueError):
                analyze_samples([], "monitoring", config)


class SimulatorTests(unittest.TestCase):
    def test_all_scenarios_reproducible_and_faults_detected(self):
        for scenario in SCENARIOS:
            with self.subTest(scenario=scenario):
                first = generate_samples(scenario, 20, 42)
                self.assertEqual(first, generate_samples(scenario, 20, 42))
                self.assertTrue(all(math.isfinite(s["roll_deg"]) for s in first))
                result = analyze_samples(first, "guided_motion")
                self.assertEqual(result["status"], "pass" if scenario == "normal" else "fail")
        self.assertNotEqual(generate_samples(seed=42), generate_samples(seed=43))

    def test_invalid_generator_settings_raise(self):
        for args in (("bad", 20, 42), ("normal", 19, 42), ("normal", True, 42),
                     ("normal", 20001, 42), ("normal", 20, "42")):
            with self.subTest(args=args), self.assertRaises(ValueError):
                generate_samples(*args)


if __name__ == "__main__":
    unittest.main()
