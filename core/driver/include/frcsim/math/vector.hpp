// Copyright (c) RenSim contributors.
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

struct alignas(16) Vector3 {
  double x, y, z;

  // Constructors
  constexpr Vector3() noexcept : x(0.0), y(0.0), z(0.0) {}
  constexpr Vector3(double x_, double y_, double z_) noexcept
      : x(x_), y(y_), z(z_) {}
  constexpr Vector3(const Vector3&) noexcept = default;
  constexpr Vector3(Vector3&&) noexcept = default;
  constexpr Vector3& operator=(const Vector3&) noexcept = default;
  constexpr Vector3& operator=(Vector3&&) noexcept = default;

  // Arithmetic operators
  constexpr Vector3 operator+(const Vector3& o) const noexcept {
    return {x + o.x, y + o.y, z + o.z};
  }
  constexpr Vector3 operator-(const Vector3& o) const noexcept {
    return {x - o.x, y - o.y, z - o.z};
  }
  constexpr Vector3 operator*(double s) const noexcept {
    return {x * s, y * s, z * s};
  }
  constexpr Vector3 operator/(double s) const noexcept {
    return (std::abs(s) > std::numeric_limits<double>::epsilon())
               ? Vector3{x / s, y / s, z / s}
               : Vector3{};
  }
  Vector3& operator+=(const Vector3& o) noexcept {
    x += o.x;
    y += o.y;
    z += o.z;
    return *this;
  }
  Vector3& operator-=(const Vector3& o) noexcept {
    x -= o.x;
    y -= o.y;
    z -= o.z;
    return *this;
  }
  Vector3& operator*=(double s) noexcept {
    x *= s;
    y *= s;
    z *= s;
    return *this;
  }
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

  // Comparison operators
  constexpr bool operator==(const Vector3& o) const noexcept {
    return x == o.x && y == o.y && z == o.z;
  }
  constexpr bool operator!=(const Vector3& o) const noexcept {
    return !(*this == o);
  }

  // Norms
  constexpr double norm2() const noexcept { return x * x + y * y + z * z; }
  double norm() const noexcept { return std::sqrt(norm2()); }
  Vector3 normalized() const noexcept {
    double n = norm();
    return (n > std::numeric_limits<double>::epsilon()) ? (*this) / n
                                                        : Vector3{};
  }
  constexpr bool isZero(double eps = 1e-12) const noexcept {
    return norm2() < eps * eps;
  }
  bool hasNaN() const noexcept {
    return std::isnan(x) || std::isnan(y) || std::isnan(z);
  }
  static Vector3 fromArray(const double arr[3]) noexcept {
    return Vector3(arr[0], arr[1], arr[2]);
  }
  void toArray(double arr[3]) const noexcept {
    arr[0] = x;
    arr[1] = y;
    arr[2] = z;
  }
  static double distance(const Vector3& a, const Vector3& b) noexcept {
    return (a - b).norm();
  }
  Vector3 clamp(const Vector3& min, const Vector3& max) const noexcept {
    return Vector3(std::max(min.x, std::min(x, max.x)),
                   std::max(min.y, std::min(y, max.y)),
                   std::max(min.z, std::min(z, max.z)));
  }
  static Vector3 lerp(const Vector3& a, const Vector3& b, double t) noexcept {
    return a * (1.0 - t) + b * t;
  }

  // Dot and cross products
  constexpr double dot(const Vector3& o) const noexcept {
    return x * o.x + y * o.y + z * o.z;
  }
  Vector3 cross(const Vector3& o) const noexcept {
    return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
  }

  // Planar helpers
  double planarSpeed() const noexcept { return std::sqrt(x * x + y * y); }
  constexpr Vector3 xy() const noexcept { return {x, y, 0.0}; }
  Vector3 planarDir() const noexcept {
    double mag = planarSpeed();
    return (mag > std::numeric_limits<double>::epsilon())
               ? Vector3{x / mag, y / mag, 0.0}
               : Vector3{};
  }

  // Projection onto another vector
  Vector3 projectOnto(const Vector3& axis) const noexcept {
    double denom = axis.norm2();
    if (denom < std::numeric_limits<double>::epsilon())
      return Vector3{};
    return axis * (dot(axis) / denom);
  }

  // Reflection across a plane normal (normalized)
  Vector3 reflect(const Vector3& n) const noexcept {
    return *this - n * (2.0 * dot(n));
  }

  // Torque at a point
  Vector3 torque(const Vector3& r) const noexcept { return r.cross(*this); }

  // Magnus force (omega x v)
  // velocity: linear velocity (m/s), omega: angular velocity (rad/s), k: Magnus
  // coefficient (SI)
  static Vector3 magnusForce(const Vector3& velocity, const Vector3& omega,
                             double k = 1e-4) noexcept {
    return omega.cross(velocity) * k;
  }

  // Drag force (-1/2 * rho * Cd * A * |v|^2 * v_hat)
  // v: velocity (m/s), Cd: drag coefficient, A: area (m^2), rho: air density
  // (kg/m^3)
  struct DragVector {
    double x{0.0};
    double y{0.0};
    double z{0.0};

    bool isZero(double eps = 1e-12) const noexcept {
      return std::abs(x) < eps && std::abs(y) < eps && std::abs(z) < eps;
    }
  };

  struct DragForceDetails {
    DragVector force{};
    DragVector direction{};
    double speed_mps{0.0};
    double speed_squared_mps2{0.0};
    double dynamic_pressure_pa{0.0};
    double drag_coefficient{0.0};
    double reference_area_m2{0.0};
    double cross_section_area_m2{0.0};
    double air_density_kgpm3{0.0};
    double linear_drag_coefficient_n_per_mps{0.0};
    double quadratic_drag_coefficient_n_per_mps2{0.0};
    double drag_force_magnitude_n{0.0};
    bool valid{false};
  };

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

  static Vector3 dragForce(const Vector3& v, double Cd, double A,
                           double rho = 1.225) noexcept {
    const auto details = dragForceDetailed(v, Cd, A, rho);
    return Vector3(details.force.x, details.force.y, details.force.z);
  }

  // Dynamic gravity (optionally with Magnus effect)
  // velocity: object velocity (m/s), spin: angular velocity (rad/s), g: gravity
  // (m/s^2), magnusCoeff: Magnus effect, gravityEffect: multiplier
  static Vector3 dynamicGravity(const Vector3& velocity, const Vector3& spin,
                                double g = 9.81, double magnusCoeff = 1e-4,
                                double gravityEffect = 1.0) noexcept {
    Vector3 magnus = magnusForce(velocity, spin, magnusCoeff);
    return Vector3(0.0, 0.0, -g * gravityEffect) + magnus;
  }

  // Zero vector constant
  static constexpr Vector3 zero() noexcept { return Vector3(0.0, 0.0, 0.0); }

  // Negation
  constexpr Vector3 operator-() const noexcept { return Vector3(-x, -y, -z); }

  // Element access (asserts in debug, undefined in release if out-of-bounds)
  double& operator[](size_t i) {
    assert(i < 3 && "Vector3 index out of bounds");
    return i == 0 ? x : i == 1 ? y : z;
  }
  const double& operator[](size_t i) const {
    assert(i < 3 && "Vector3 index out of bounds");
    return i == 0 ? x : i == 1 ? y : z;
  }

  // Output stream
  friend std::ostream& operator<<(std::ostream& os, const Vector3& v) {
    os << std::fixed << std::setprecision(4) << "[" << v.x << ", " << v.y
       << ", " << v.z << "]";
    return os;
  }

  // Unit vectors
  static constexpr Vector3 unitX() noexcept { return Vector3(1.0, 0.0, 0.0); }
  static constexpr Vector3 unitY() noexcept { return Vector3(0.0, 1.0, 0.0); }
  static constexpr Vector3 unitZ() noexcept { return Vector3(0.0, 0.0, 1.0); }

  // Traction/grip (friction force)
  // normal: surface normal (should be normalized), frictionCoeff: coefficient
  // of friction, normalForce: magnitude of normal force
  static Vector3 tractionForce(const Vector3& normal, double frictionCoeff,
                               double normalForce) noexcept {
    return normal * (frictionCoeff * normalForce);
  }
};

// Scalar multiplication (left)
inline Vector3 operator*(double s, const Vector3& v) noexcept {
  return v * s;
}

}  // namespace frcsim
