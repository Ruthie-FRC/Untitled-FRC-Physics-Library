from __future__ import annotations

from typing import Tuple


Vec2 = Tuple[float, float]


def add(a: Vec2, b: Vec2) -> Vec2:
    return (a[0] + b[0], a[1] + b[1])


def sub(a: Vec2, b: Vec2) -> Vec2:
    return (a[0] - b[0], a[1] - b[1])


def mul(a: Vec2, scalar: float) -> Vec2:
    return (a[0] * scalar, a[1] * scalar)


def dot(a: Vec2, b: Vec2) -> float:
    return a[0] * b[0] + a[1] * b[1]


def norm(a: Vec2) -> float:
    return (a[0] * a[0] + a[1] * a[1]) ** 0.5


def normalized(a: Vec2) -> Vec2:
    n = norm(a)
    if n < 1e-12:
        return (1.0, 0.0)
    return (a[0] / n, a[1] / n)
