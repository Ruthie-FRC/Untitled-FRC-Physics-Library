from __future__ import annotations

from pathlib import Path
import json

from math_primitives import dot


ROOT = Path(__file__).resolve().parents[2]
FIXTURE_PATH = ROOT / "vendordep" / "src" / "test" / "resources" / "math" / "cpp_math_contract.json"


def _cross(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _norm3(a: tuple[float, float, float]) -> float:
    return (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]) ** 0.5


def _normalize3(a: tuple[float, float, float]) -> tuple[float, float, float]:
    n = _norm3(a)
    if n < 1e-12:
        return (0.0, 0.0, 0.0)
    return (a[0] / n, a[1] / n, a[2] / n)


def _close(a: float, b: float, eps: float) -> bool:
    return abs(a - b) <= eps


def validate_math_fixture() -> dict[str, float]:
    raw = json.loads(FIXTURE_PATH.read_text(encoding="utf-8"))
    eps = float(raw["epsilon"])

    a = tuple(raw["vector"]["a"])
    b = tuple(raw["vector"]["b"])
    expected_dot = float(raw["vector"]["dot"])
    expected_cross = tuple(raw["vector"]["cross"])
    expected_norm = float(raw["vector"]["norm_a"])
    expected_normalized = tuple(raw["vector"]["normalized_a"])

    computed_dot = dot((a[0], a[1]), (b[0], b[1])) + (a[2] * b[2])
    computed_cross = _cross(a, b)
    computed_norm = _norm3(a)
    computed_normalized = _normalize3(a)

    if not _close(computed_dot, expected_dot, eps):
        raise ValueError("Dot product mismatch vs C++ contract")
    for i in range(3):
        if not _close(computed_cross[i], expected_cross[i], eps):
            raise ValueError("Cross product mismatch vs C++ contract")
        if not _close(computed_normalized[i], expected_normalized[i], eps):
            raise ValueError("Normalized vector mismatch vs C++ contract")
    if not _close(computed_norm, expected_norm, eps):
        raise ValueError("Vector norm mismatch vs C++ contract")

    return {
        "dot": computed_dot,
        "norm_a": computed_norm,
        "cross_z": computed_cross[2],
    }


if __name__ == "__main__":
    summary = validate_math_fixture()
    print("Math fixture validated:", summary)
