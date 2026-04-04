from __future__ import annotations

from pathlib import Path
import json

from telemetry_schema import SensorPacket
from wpilib_bridge import flatten_for_networktables


ROOT = Path(__file__).resolve().parents[2]
FIXTURE_PATH = ROOT / "vendordep" / "src" / "test" / "resources" / "telemetry" / "golden_packets.jsonl"


def validate_fixture() -> dict[str, float]:
    if not FIXTURE_PATH.exists():
        raise FileNotFoundError(f"Fixture not found: {FIXTURE_PATH}")

    packets: list[SensorPacket] = []
    for line in FIXTURE_PATH.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        raw = json.loads(line)
        packets.append(SensorPacket.from_dict(raw))

    if len(packets) < 2:
        raise ValueError("Expected at least two packets in golden fixture")

    first_flat = flatten_for_networktables(packets[0])
    last_flat = flatten_for_networktables(packets[-1])

    if first_flat["sim/tick"] != 1.0:
        raise ValueError("Expected first sim/tick to be 1.0")
    if last_flat["sim/contact_count"] != 1.0:
        raise ValueError("Expected last sim/contact_count to be 1.0")

    return {
        "frames": float(len(packets)),
        "first_tick": first_flat["sim/tick"],
        "last_tick": last_flat["sim/tick"],
        "last_contacts": last_flat["sim/contact_count"],
    }


if __name__ == "__main__":
    summary = validate_fixture()
    print("Telemetry fixture validated:", summary)
