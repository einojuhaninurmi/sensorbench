"""Quaternion conversion for the BNO085 ingestion adapter.

The public argument order is scalar first: ``(w, x, y, z)``. Adafruit names
these components ``(real, i, j, k)``; a serial stream printed as i,j,k,real
must therefore be reordered before calling this function.

We use right-handed active rotations with intrinsic ZYX yaw-pitch-roll,
equivalently R = Rz(yaw) Ry(pitch) Rx(roll), matching Adafruit's example:
https://github.com/adafruit/Adafruit_BNO08x/blob/master/examples/quaternion_yaw_pitch_roll/quaternion_yaw_pitch_roll.ino
Component naming is documented in its src/sh2_SensorValue.h.

Roll and yaw are returned in [-180, 180), pitch in [-90, 90], all in degrees.
At pitch +/-90 the decomposition is not unique: this implementation chooses
roll=0 and the equivalent yaw. No mounting transform or calibrated heading
accuracy is implied by the conversion.
"""

from __future__ import annotations

import math


def _finite_component(value, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"Quaternion {name} must be a finite number, not a boolean")
    try:
        result = float(value)
    except (OverflowError, ValueError) as exc:
        raise ValueError(f"Quaternion {name} must be a finite number") from exc
    if not math.isfinite(result):
        raise ValueError(f"Quaternion {name} must be a finite number")
    return result


def _wrapped_degrees(radians: float) -> float:
    result = (math.degrees(radians) + 180.0) % 360.0 - 180.0
    return 0.0 if result == 0.0 else result


def quaternion_to_euler(w, x, y, z) -> dict:
    """Normalize a finite nonzero quaternion and return roll, pitch and yaw.

    Scaling before computing the norm avoids overflow for large components and
    underflow for tiny ones. A canonical sign makes q and -q produce identical
    results, including half-turn and signed-zero boundaries. Original sensor
    components should be preserved separately as acquisition evidence.
    """
    components = [_finite_component(value, name) for value, name in zip((w, x, y, z), ("w", "x", "y", "z"))]
    scale = max(abs(value) for value in components)
    if scale == 0.0:
        raise ValueError("A zero quaternion does not define an orientation")
    scaled = [value / scale for value in components]
    norm = math.sqrt(sum(value * value for value in scaled))
    components = [value / norm for value in scaled]
    first_nonzero = next(value for value in components if value != 0.0)
    if first_nonzero < 0.0:
        components = [-value for value in components]
    w, x, y, z = [0.0 if value == 0.0 else value for value in components]

    sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    # Within floating-point roundoff of the singularity, choose a stable
    # representative instead of allowing two near-zero atan2 inputs to pick
    # arbitrary roll/yaw values. Away from it retain the regular decomposition.
    if 1.0 - abs(sin_pitch) <= 1e-15:
        pitch = math.copysign(90.0, sin_pitch)
        roll = 0.0
        yaw = _wrapped_degrees(2.0 * math.atan2(z, w))
    else:
        pitch = math.degrees(math.asin(sin_pitch))
        roll = _wrapped_degrees(math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)))
        yaw = _wrapped_degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
    return {"roll_deg": roll, "pitch_deg": 0.0 if pitch == 0.0 else pitch, "yaw_deg": yaw}
