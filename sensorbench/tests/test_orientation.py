"""Known rotations, acquisition order and numerical boundaries for IMU data."""

import math
import random
import unittest

from sensorbench.orientation import quaternion_to_euler


def quaternion_from_zyx(roll_deg, pitch_deg, yaw_deg):
    """Construct test rotations independently by composing half-angle terms."""
    roll, pitch, yaw = [math.radians(value) / 2 for value in (roll_deg, pitch_deg, yaw_deg)]
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)


class QuaternionTests(unittest.TestCase):
    def assert_euler(self, actual, roll, pitch, yaw, delta=1e-8):
        self.assertEqual(set(actual), {"roll_deg", "pitch_deg", "yaw_deg"})
        for field, expected in zip(("roll_deg", "pitch_deg", "yaw_deg"), (roll, pitch, yaw)):
            self.assertAlmostEqual(actual[field], expected, delta=delta, msg=field)

    def test_identity(self):
        self.assert_euler(quaternion_to_euler(1, 0, 0, 0), 0, 0, 0)

    def test_positive_and_negative_roll_quarter_turns(self):
        root_half = math.sqrt(0.5)
        for sign in (-1, 1):
            with self.subTest(sign=sign):
                self.assert_euler(quaternion_to_euler(root_half, sign * root_half, 0, 0), sign * 90, 0, 0)

    def test_positive_and_negative_yaw_quarter_turns(self):
        root_half = math.sqrt(0.5)
        for sign in (-1, 1):
            with self.subTest(sign=sign):
                self.assert_euler(quaternion_to_euler(root_half, 0, 0, sign * root_half), 0, 0, sign * 90)

    def test_pitch_and_combined_known_zyx_rotations(self):
        for roll, pitch, yaw in ((0, 45, 0), (0, -45, 0), (30, 20, 40), (-40, -25, -70), (170, 60, -179)):
            with self.subTest(angles=(roll, pitch, yaw)):
                self.assert_euler(quaternion_to_euler(*quaternion_from_zyx(roll, pitch, yaw)), roll, pitch, yaw)

    def test_pitch_singularities_have_stable_equivalent_representation(self):
        self.assert_euler(quaternion_to_euler(*quaternion_from_zyx(0, 90, 0)), 0, 90, 0)
        self.assert_euler(quaternion_to_euler(*quaternion_from_zyx(0, -90, 0)), 0, -90, 0)
        self.assert_euler(quaternion_to_euler(*quaternion_from_zyx(30, 90, 40)), 0, 90, 10)
        self.assert_euler(quaternion_to_euler(*quaternion_from_zyx(30, -90, 40)), 0, -90, 70)

    def test_near_singular_pitch_is_not_unnecessarily_snapped_to_ninety(self):
        for pitch in (89.99, -89.99):
            with self.subTest(pitch=pitch):
                self.assert_euler(quaternion_to_euler(*quaternion_from_zyx(10, pitch, 30)), 10, pitch, 30, delta=1e-6)

    def test_scaled_inputs_do_not_overflow_or_underflow_normalization(self):
        base = quaternion_from_zyx(30, 20, 40)
        for scale in (2, 1e-300, 1e308):
            with self.subTest(scale=scale):
                self.assert_euler(quaternion_to_euler(*(value * scale for value in base)), 30, 20, 40)
        self.assert_euler(quaternion_to_euler(1e308, 1e308, 1e308, 1e308), 90, 0, 90)

    def test_two_decimal_serial_values_are_normalized_without_claiming_lost_precision(self):
        # A rounded +45-degree roll quaternion is not exactly unit length.
        result = quaternion_to_euler(0.92, 0.38, 0, 0)
        expected_roll = math.degrees(2 * math.atan2(0.38, 0.92))
        self.assert_euler(result, expected_roll, 0, 0)
        self.assertLess(abs(result["roll_deg"] - 45), 1)

    def test_q_and_negated_q_are_identical_including_half_turns(self):
        quaternions = [(1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 0, 1), quaternion_from_zyx(30, 90, 40)]
        rng = random.Random(42)
        quaternions.extend(tuple(rng.uniform(-2, 2) for _ in range(4)) for _ in range(50))
        for values in quaternions:
            with self.subTest(values=values):
                self.assertEqual(quaternion_to_euler(*values), quaternion_to_euler(*(-v for v in values)))

    def test_adafruit_ijk_real_stream_is_reordered_into_wxyz(self):
        # Source stream order is i,j,k,real for a +90-degree yaw.
        i, j, k, real = 0, 0, math.sqrt(0.5), math.sqrt(0.5)
        self.assert_euler(quaternion_to_euler(w=real, x=i, y=j, z=k), 0, 0, 90)

    def test_zero_nonfinite_boolean_and_non_numeric_inputs_raise(self):
        with self.assertRaises(ValueError):
            quaternion_to_euler(0, 0, 0, 0)
        for bad in (True, False, None, "1", math.nan, math.inf, -math.inf, 10 ** 1000):
            for index in range(4):
                values = [1, 0, 0, 0]
                values[index] = bad
                with self.subTest(value=bad, index=index), self.assertRaises(ValueError):
                    quaternion_to_euler(*values)

    def test_outputs_stay_finite_and_within_quality_protocol_bounds(self):
        rng = random.Random(123)
        for _ in range(100):
            result = quaternion_to_euler(*(rng.uniform(-1, 1) for _ in range(4)))
            self.assertTrue(all(isinstance(value, float) and math.isfinite(value) for value in result.values()))
            self.assertTrue(-180 <= result["roll_deg"] < 180)
            self.assertTrue(-90 <= result["pitch_deg"] <= 90)
            self.assertTrue(-180 <= result["yaw_deg"] < 180)


if __name__ == "__main__":
    unittest.main()
