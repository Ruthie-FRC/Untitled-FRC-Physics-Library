// Copyright (c) RenSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once
#include <cmath>
#include <cassert>
#include <limits>
#include <iostream>
#include <iomanip>

#include "vector.hpp"
#include "quaternion.hpp"

namespace frcsim {

struct alignas(16) Matrix3 {
  double m[3][3];  // row-major

  /* Constructors */

  constexpr Matrix3() noexcept : m{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}} {}

  constexpr Matrix3(double m00, double m01, double m02, double m10, double m11,
                    double m12, double m20, double m21, double m22) noexcept
      : m{{m00, m01, m02}, {m10, m11, m12}, {m20, m21, m22}} {}

  static constexpr Matrix3 identity() noexcept { return Matrix3(); }

  static constexpr Matrix3 zero() noexcept {
    return Matrix3(0, 0, 0, 0, 0, 0, 0, 0, 0);
  }

  /* Element access */

  double* operator[](size_t i) {
    assert(i < 3);
    return m[i];
  }

  const double* operator[](size_t i) const {
    assert(i < 3);
    return m[i];
  }

  /* Matrix arithmetic */

  constexpr Matrix3 operator+(const Matrix3& o) const noexcept {
    Matrix3 r = zero();

    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        r.m[i][j] = m[i][j] + o.m[i][j];

    return r;
  }

  constexpr Matrix3 operator-(const Matrix3& o) const noexcept {
    Matrix3 r = zero();

    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        r.m[i][j] = m[i][j] - o.m[i][j];

    return r;
  }

  constexpr Matrix3 operator*(double s) const noexcept {
    Matrix3 r = zero();

    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        r.m[i][j] = m[i][j] * s;

    return r;
  }

  /* Matrix × Vector */

  Vector3 operator*(const Vector3& v) const noexcept {
    return Vector3(m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
                   m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
                   m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z);
  }

  /* Matrix × Matrix */

  Matrix3 operator*(const Matrix3& o) const noexcept {
    Matrix3 r = zero();

    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        for (int k = 0; k < 3; k++)
          r.m[i][j] += m[i][k] * o.m[k][j];

    return r;
  }

  /* Transpose */

  Matrix3 transpose() const noexcept {
    return Matrix3(m[0][0], m[1][0], m[2][0], m[0][1], m[1][1], m[2][1],
                   m[0][2], m[1][2], m[2][2]);
  }

  /* Determinant */

  double determinant() const noexcept {
    return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
           m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
           m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
  }

  /* Inverse */

  bool tryInverse(
      Matrix3& out,
      double eps = std::numeric_limits<double>::epsilon()) const noexcept {
    double det = determinant();

    if (std::abs(det) < eps)
      return false;

    double invDet = 1.0 / det;

    Matrix3 r;

    r.m[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * invDet;
    r.m[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * invDet;
    r.m[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * invDet;

    r.m[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * invDet;
    r.m[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * invDet;
    r.m[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * invDet;

    r.m[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * invDet;
    r.m[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * invDet;
    r.m[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * invDet;

    out = r;
    return true;
  }

  Matrix3 inverse() const noexcept {
    Matrix3 r;
    if (!tryInverse(r))
      return identity();  // safe fallback for legacy callers

    return r;
  }

  /* Rotation helper */

  static Matrix3 fromQuaternion(const Quaternion& q) noexcept {
    Matrix3 r;

    q.toMatrix(r.m);

    return r;
  }

  /* Comparison */

  bool operator==(const Matrix3& o) const noexcept {
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        if (m[i][j] != o.m[i][j])
          return false;

    return true;
  }

  bool operator!=(const Matrix3& o) const noexcept { return !(*this == o); }

  /* Debug output */

  friend std::ostream& operator<<(std::ostream& os, const Matrix3& mat) {
    os << std::fixed << std::setprecision(4);

    for (int i = 0; i < 3; i++)
      os << "[" << mat.m[i][0] << " " << mat.m[i][1] << " " << mat.m[i][2]
         << "]\n";

    return os;
  }
};

/* Scalar multiply (left) */

inline Matrix3 operator*(double s, const Matrix3& m) noexcept {
  return m * s;
}

}  // namespace frcsim
