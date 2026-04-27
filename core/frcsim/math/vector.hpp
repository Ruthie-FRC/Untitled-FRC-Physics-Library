// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once
#include <cmath>
#include <algorithm>
#include <iostream>
#include <cassert>
#include <limits>
#include <iomanip>

namespace frcsim {

/**
 * @brief 3D vector utility used throughout JSim physics.
 *
 * This type stores Cartesian coordinates in SI units when used with simulation
 * state (meters, meters/second, radians/second, newtons, etc.). The math
 * helpers are unit-agnostic, but callers should keep units consistent.
 */
struct alignas(16) Vector3 {
  /** @brief X component. */
  double x;
  /** @brief Y component. */
  double y;
  /** @brief Z component. */
  double z;

  /** @brief Constructs a zero vector. */
  constexpr Vector3() noexcept : x(0.0), y(0.0), z(0.0) {}
  /**
   * @brief Constructs a vector from component values.
   * @param x_ X component.
   * @param y_ Y component.
   * @param z_ Z component.
   */
  constexpr Vector3(double x_, double y_, double z_) noexcept
      : x(x_), y(y_), z(z_) {}
  constexpr Vector3(const Vector3&) noexcept = default;
  constexpr Vector3(Vector3&&) noexcept = default;
  constexpr Vector3& operator=(const Vector3&) noexcept = default;
  constexpr Vector3& operator=(Vector3&&) noexcept = default;

  /** @brief Returns component-wise sum. */
  constexpr Vector3 operator+(const Vector3& o) const noexcept {
    return {x + o.x, y + o.y, z + o.z};
  }
  /** @brief Returns component-wise difference. */
  constexpr Vector3 operator-(const Vector3& o) const noexcept {
    return {x - o.x, y - o.y, z - o.z};
  }
  /** @brief Returns this vector scaled by a scalar. */
  constexpr Vector3 operator*(double s) const noexcept {
    return {x * s, y * s, z * s};
  }
  /**
   * @brief Returns this vector divided by a scalar.
   * @note Returns zero when `s` is effectively zero.
   */
  constexpr Vector3 operator/(double s) const noexcept {
    return (std::abs(s) > std::numeric_limits<double>::epsilon())
               ? Vector3{x / s, y / s, z / s}
               : Vector3{};
  }
  /** @brief Adds `o` component-wise into this vector. */
  Vector3& operator+=(const Vector3& o) noexcept {
    x += o.x;
    y += o.y;
    z += o.z;
    return *this;
  }
  /** @brief Subtracts `o` component-wise from this vector. */
  Vector3& operator-=(const Vector3& o) noexcept {
    x -= o.x;
    y -= o.y;
    z -= o.z;
    return *this;
  }
  /** @brief Multiplies each component by `s`. */
  Vector3& operator*=(double s) noexcept {
    x *= s;
    y *= s;
    z *= s;
    return *this;
  }
  /**
   * @brief Divides each component by `s`.
   * @note Sets the vector to zero when `s` is effectively zero.
   */
  Vector3& operator/=(double s) noexcept {
    if (std::abs(s) > std::numeric_limits<double>::epsilon()) {
      x /= s;
      y /= s;
      z /= s;
    } else {
      x = y = z = 0.0;
    }
    return *this;
  }

  /** @brief Exact component-wise equality comparison. */
  constexpr bool operator==(const Vector3& o) const noexcept {
    return x == o.x && y == o.y && z == o.z;
  }
  /** @brief Exact component-wise inequality comparison. */
  constexpr bool operator!=(const Vector3& o) const noexcept {
    return !(*this == o);
  }

  /** @brief Returns squared magnitude: $x^2+y^2+z^2$. */
  constexpr double norm2() const noexcept { return x * x + y * y + z * z; }
  /** @brief Returns Euclidean magnitude. */
  double norm() const noexcept { return std::sqrt(norm2()); }
  /**
   * @brief Returns the normalized direction vector.
   * @return Unit vector in the same direction, or zero for near-zero input.
   */
  Vector3 normalized() const noexcept {
    double n = norm();
    return (n > std::numeric_limits<double>::epsilon()) ? (*this) / n
                                                        : Vector3{};
  }
  /**
   * @brief Checks whether the vector is near zero magnitude.
   * @param eps Absolute tolerance on magnitude.
   */
  constexpr bool isZero(double eps = 1e-12) const noexcept {
    return norm2() < eps * eps;
  }
  /** @brief Returns true when any component is NaN. */
  bool hasNaN() const noexcept {
    return std::isnan(x) || std::isnan(y) || std::isnan(z);
  }
  /** @brief Builds a vector from a 3-element array. */
  static Vector3 fromArray(const double arr[3]) noexcept {
    return Vector3(arr[0], arr[1], arr[2]);
  }
  /** @brief Writes components to a 3-element array. */
  void toArray(double arr[3]) const noexcept {
    arr[0] = x;
    arr[1] = y;
    arr[2] = z;
  }
  /** @brief Returns Euclidean distance between two vectors treated as points. */
  static double distance(const Vector3& a, const Vector3& b) noexcept {
    return (a - b).norm();
  }
  /**
   * @brief Clamps each component into the inclusive `[min, max]` range.
   */
  Vector3 clamp(const Vector3& min, const Vector3& max) const noexcept {
    return Vector3(std::max(min.x, std::min(x, max.x)),
                   std::max(min.y, std::min(y, max.y)),
                   std::max(min.z, std::min(z, max.z)));
  }
  /**
   * @brief Linearly interpolates from `a` to `b`.
   * @param a Start vector.
   * @param b End vector.
   * @param t Interpolation ratio where 0 is `a` and 1 is `b`.
   */
  static Vector3 lerp(const Vector3& a, const Vector3& b, double t) noexcept {
    return a * (1.0 - t) + b * t;
  }

  /** @brief Returns dot product with `o`. */
  constexpr double dot(const Vector3& o) const noexcept {
    return x * o.x + y * o.y + z * o.z;
  }
  /** @brief Returns cross product with `o`. */
  Vector3 cross(const Vector3& o) const noexcept {
    return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
  }

  /** @brief Returns magnitude of the XY projection. */
  double planarSpeed() const noexcept { return std::sqrt(x * x + y * y); }
  /** @brief Returns this vector projected onto the XY plane (`z=0`). */
  constexpr Vector3 xy() const noexcept { return {x, y, 0.0}; }
  /**
   * @brief Returns normalized XY direction with `z=0`.
   * @return Unit planar direction, or zero when planar magnitude is near zero.
   */
  Vector3 planarDir() const noexcept {
    double mag = planarSpeed();
    return (mag > std::numeric_limits<double>::epsilon())
               ? Vector3{x / mag, y / mag, 0.0}
               : Vector3{};
  }

  /**
   * @brief Projects this vector onto another axis.
   * @param axis Axis to project onto.
   * @return Vector component of this vector along axis.
   */
  Vector3 projectOnto(const Vector3& axis) const noexcept {
    double denom = axis.norm2();
    if (denom < std::numeric_limits<double>::epsilon())
      return Vector3{};
    return axis * (dot(axis) / denom);
  }

  /**
   * @brief Reflects this vector across a normalized plane normal.
   * @param n Unit normal of the reflection plane.
   * @return Reflected vector.
   */
  Vector3 reflect(const Vector3& n) const noexcept {
    return *this - n * (2.0 * dot(n));
  }

  /**
   * @brief Computes torque from a force applied at offset r.
   * @param r Lever arm from the reference point to the application point.
   * @return Torque vector r x F.
   */
  Vector3 torque(const Vector3& r) const noexcept { return r.cross(*this); }

  /**
   * @brief Computes the Magnus lift force omega x v scaled by k.
   * @param velocity Linear velocity in meters per second.
   * @param omega Angular velocity in radians per second.
   * @param k Magnus coefficient in SI units.
   * @return Lift force vector in newtons.
   */
  static Vector3 magnusForce(const Vector3& velocity, const Vector3& omega,
                             double k = 1e-4) noexcept {
    return omega.cross(velocity) * k;
  }

  /**
   * @brief Directional drag force vector and validity metadata.
   */
  struct DragVector {
    /** @brief X component. */
    double x{0.0};
    /** @brief Y component. */
    double y{0.0};
    /** @brief Z component. */
    double z{0.0};

    /** @brief Returns true when all components are within `eps` of zero. */
    bool isZero(double eps = 1e-12) const noexcept {
      return std::abs(x) < eps && std::abs(y) < eps && std::abs(z) < eps;
    }
  };

  /** @brief Expanded drag diagnostics for force computation. */
  struct DragForceDetails {
    /** @brief Drag force vector in newtons. */
    DragVector force{};
    /** @brief Unit direction of velocity, undefined when speed is zero. */
    DragVector direction{};
    /** @brief Speed magnitude in meters per second. */
    double speed_mps{0.0};
    /** @brief Squared speed in $(m/s)^2$. */
    double speed_squared_mps2{0.0};
    /** @brief Dynamic pressure in pascals. */
    double dynamic_pressure_pa{0.0};
    /** @brief Drag coefficient used in the calculation. */
    double drag_coefficient{0.0};
    /** @brief Reference area in square meters. */
    double reference_area_m2{0.0};
    /** @brief Effective cross-sectional area in square meters. */
    double cross_section_area_m2{0.0};
    /** @brief Air density in kilograms per cubic meter. */
    double air_density_kgpm3{0.0};
    /** @brief Linear drag term coefficient in N/(m/s). */
    double linear_drag_coefficient_n_per_mps{0.0};
    /** @brief Quadratic drag term coefficient in N/(m/s)^2. */
    double quadratic_drag_coefficient_n_per_mps2{0.0};
    /** @brief Total drag magnitude in newtons. */
    double drag_force_magnitude_n{0.0};
    /** @brief True when the provided inputs were valid for drag evaluation. */
    bool valid{false};
  };

  /**
   * @brief Computes drag force details for a body moving through air.
   * @param v Body velocity in meters per second.
   * @param Cd Drag coefficient.
   * @param A Reference area in square meters.
   * @param rho Air density in kilograms per cubic meter.
   * @param linear_drag_coefficient_n_per_mps Optional linear drag term.
   * @return Detailed drag force breakdown and validity state.
   */
  static DragForceDetails dragForceDetailed(
      const Vector3& v, double Cd, double A, double rho = 1.225,
      double linear_drag_coefficient_n_per_mps = 0.0) noexcept {
    DragForceDetails details{};
    details.drag_coefficient = Cd;
    details.reference_area_m2 = A;
    details.cross_section_area_m2 = A;
    details.air_density_kgpm3 = rho;
    details.linear_drag_coefficient_n_per_mps =
        linear_drag_coefficient_n_per_mps;
    details.quadratic_drag_coefficient_n_per_mps2 = 0.5 * rho * Cd * A;

    if (!std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z) ||
        !std::isfinite(Cd) || !std::isfinite(A) || !std::isfinite(rho) ||
        !std::isfinite(linear_drag_coefficient_n_per_mps)) {
      return details;
    }

    details.speed_mps = v.norm();
    details.speed_squared_mps2 = details.speed_mps * details.speed_mps;

    if (details.speed_mps <= std::numeric_limits<double>::epsilon() ||
        Cd <= 0.0 || A <= 0.0 || rho <= 0.0 ||
        linear_drag_coefficient_n_per_mps < 0.0) {
      return details;
    }

    details.direction = {v.x / details.speed_mps, v.y / details.speed_mps,
                         v.z / details.speed_mps};
    details.dynamic_pressure_pa = 0.5 * rho * details.speed_squared_mps2;
    const double linear_force_n =
        linear_drag_coefficient_n_per_mps * details.speed_mps;
    const double quadratic_force_n = details.dynamic_pressure_pa * Cd * A;
    const double force_magnitude_n = linear_force_n + quadratic_force_n;
    details.drag_force_magnitude_n = force_magnitude_n;
    details.force = {details.direction.x * (-force_magnitude_n),
                     details.direction.y * (-force_magnitude_n),
                     details.direction.z * (-force_magnitude_n)};
    details.valid = true;
    return details;
  }

  /**
   * @brief Computes the total drag force vector for a velocity and body.
   * @param v Body velocity in meters per second.
   * @param Cd Drag coefficient.
   * @param A Reference area in square meters.
   * @param rho Air density in kilograms per cubic meter.
   * @return Drag force vector in newtons.
   */
  static Vector3 dragForce(const Vector3& v, double Cd, double A,
                           double rho = 1.225) noexcept {
    const auto details = dragForceDetailed(v, Cd, A, rho);
    return Vector3(details.force.x, details.force.y, details.force.z);
  }

  /**
   * @brief Combines gravity with an optional Magnus lift contribution.
   * @param velocity Object velocity in meters per second.
   * @param spin Angular velocity in radians per second.
   * @param g Gravity magnitude in meters per second squared.
   * @param magnusCoeff Magnus coefficient applied to the lift term.
   * @param gravityEffect Multiplier applied to the gravity vector.
   * @return Gravity plus Magnus acceleration expressed as a force-like vector.
   */
  static Vector3 dynamicGravity(const Vector3& velocity, const Vector3& spin,
                                double g = 9.81, double magnusCoeff = 1e-4,
                                double gravityEffect = 1.0) noexcept {
    Vector3 magnus = magnusForce(velocity, spin, magnusCoeff);
    return Vector3(0.0, 0.0, -g * gravityEffect) + magnus;
  }

  /** @brief Returns the zero vector. */
  static constexpr Vector3 zero() noexcept { return Vector3(0.0, 0.0, 0.0); }

  /** @brief Returns the additive inverse of this vector. */
  constexpr Vector3 operator-() const noexcept { return Vector3(-x, -y, -z); }

  /**
   * @brief Returns a mutable component by index.
   * @param i Component index: 0 for x, 1 for y, 2 for z.
   * @return Reference to the selected component.
   */
  double& operator[](size_t i) {
    assert(i < 3 && "Vector3 index out of bounds");
    return i == 0 ? x : i == 1 ? y : z;
  }
  /**
   * @brief Returns a const component by index.
   * @param i Component index: 0 for x, 1 for y, 2 for z.
   * @return Const reference to the selected component.
   */
  const double& operator[](size_t i) const {
    assert(i < 3 && "Vector3 index out of bounds");
    return i == 0 ? x : i == 1 ? y : z;
  }

  /** @brief Streams the vector in a fixed-width bracketed format. */
  friend std::ostream& operator<<(std::ostream& os, const Vector3& v) {
    os << std::fixed << std::setprecision(4) << "[" << v.x << ", " << v.y
       << ", " << v.z << "]";
    return os;
  }

  /** @brief Returns the +X basis vector. */
  static constexpr Vector3 unitX() noexcept { return Vector3(1.0, 0.0, 0.0); }
  /** @brief Returns the +Y basis vector. */
  static constexpr Vector3 unitY() noexcept { return Vector3(0.0, 1.0, 0.0); }
  /** @brief Returns the +Z basis vector. */
  static constexpr Vector3 unitZ() noexcept { return Vector3(0.0, 0.0, 1.0); }

  /**
   * @brief Computes a traction/friction force aligned with the supplied normal.
   * @param normal Contact normal direction (expected normalized).
   * @param frictionCoeff Unitless friction coefficient.
   * @param normalForce Normal force magnitude in newtons.
   * @return Traction vector in newtons.
   */
  static Vector3 tractionForce(const Vector3& normal, double frictionCoeff,
                               double normalForce) noexcept {
    return normal * (frictionCoeff * normalForce);
  }
};

/** @brief Left scalar multiplication helper (`s * v`). */
inline Vector3 operator*(double s, const Vector3& v) noexcept {
  return v * s;
}

}  // namespace frcsim
