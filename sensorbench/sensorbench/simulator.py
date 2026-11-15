"""Deterministic simulator data for testing SensorBench, never hardware evidence."""

from __future__ import annotations

import math
import random

SCENARIOS = ("normal", "packet_loss", "frozen", "out_of_range")


def generate_samples(scenario: str = "normal", count: int = 200, seed: int = 42) -> list[dict]:
    """Generate ``count`` acquisition opportunities, preserving dropped IDs/times.

    Packet loss removes four adjacent opportunities near the middle; its output
    therefore contains ``count - 4`` records. Other scenarios contain ``count``.
    Every returned record is synthetic. ``frozen`` keeps orientation constant
    while timestamps and sequence IDs advance normally.
    """
    if scenario not in SCENARIOS:
        raise ValueError("Unknown simulator scenario")
    if isinstance(count, bool) or not isinstance(count, int) or not 20 <= count <= 20000:
        raise ValueError("count must be an integer between 20 and 20000")
    if isinstance(seed, bool) or not isinstance(seed, int):
        raise ValueError("seed must be an integer")
    rng = random.Random(seed)
    samples = []
    loss_start = max(5, count // 2)
    for seq in range(count):
        phase = 2.0 * math.pi * seq / (count - 1)
        roll = 12.0 * math.sin(phase) + rng.uniform(-0.03, 0.03)
        pitch = 7.0 * math.sin(phase / 2.0) + rng.uniform(-0.03, 0.03)
        yaw = 30.0 + 20.0 * math.sin(phase) + rng.uniform(-0.03, 0.03)
        if scenario == "packet_loss" and loss_start <= seq < loss_start + 4:
            continue
        if scenario == "frozen":
            roll, pitch, yaw = 1.0, 2.0, 30.0
        if scenario == "out_of_range" and seq == count // 2:
            pitch = 120.0
        samples.append({"seq": seq, "t_ms": seq * 20,
                        "roll_deg": round(roll, 6), "pitch_deg": round(pitch, 6), "yaw_deg": round(yaw, 6),
                        "gps_fix": True, "latitude_deg": 60.17, "longitude_deg": 24.94})
    return samples
