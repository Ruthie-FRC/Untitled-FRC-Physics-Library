#pragma once
#include <cmath>
#include <cassert>
#include "vector.hpp"

namespace frcsim {

struct Quaternion {
    alignas(16) double w, x, y, z;
    Vector3 angularVelocity; // rad/s in world frame

    constexpr Quaternion() noexcept : w(1.0), x(0.0), y(0.0), z(0.0), angularVelocity{} {}
    constexpr Quaternion(double w_, double x_, double y_, double z_) noexcept
        : w(w_), x(x_), y(y_), z(z_), angularVelocity{} {}
    constexpr Quaternion(double w_, const Vector3& v) noexcept
        : w(w_), x(v.x), y(v.y), z(v.z), angularVelocity{} {}


    [[nodiscard]] constexpr double norm2() const noexcept { return w*w + x*x + y*y + z*z; }
    [[nodiscard]] double norm() const noexcept { return std::sqrt(norm2()); }

    [[nodiscard]] Quaternion normalized() const noexcept {
        double n = norm();
        if (n < 1e-12) return Quaternion();
        return Quaternion(w/n, x/n, y/n, z/n);
    }
    bool isIdentity(double eps=1e-12) const noexcept { return std::abs(w-1.0)<eps && std::abs(x)<eps && std::abs(y)<eps && std::abs(z)<eps; }
    bool hasNaN() const noexcept { return std::isnan(w)||std::isnan(x)||std::isnan(y)||std::isnan(z); }

    // Static: from axis-angle
    static Quaternion fromAxisAngle(const Vector3& axis, double angleRad) noexcept {
        double half = 0.5*angleRad;
        double s = std::sin(half);
        return Quaternion(std::cos(half), axis.x*s, axis.y*s, axis.z*s);
    }
    // To axis-angle
    void toAxisAngle(Vector3& axis, double& angleRad) const noexcept {
        Quaternion q = normalized();
        angleRad = 2.0 * std::acos(q.w);
        double s = std::sqrt(1.0 - q.w*q.w);
        if (s < 1e-12) { axis = Vector3(1,0,0); }
        else { axis = Vector3(q.x/s, q.y/s, q.z/s); }
    }
    // Slerp
    static Quaternion slerp(const Quaternion& a, const Quaternion& b, double t) noexcept {
        double dot = a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z;
        Quaternion b2 = b;
        if (dot < 0.0) { dot = -dot; b2 = b*-1.0; }
        if (dot > 0.9995) {
            // Linear
            return (a*(1.0-t) + b2*t).normalized();
        }
        double theta = std::acos(dot);
        double s1 = std::sin((1.0-t)*theta);
        double s2 = std::sin(t*theta);
        double s = std::sin(theta);
        return (a*s1 + b2*s2) * (1.0/s);
    }
    // To 3x3 rotation matrix (row-major)
    void toMatrix(double m[3][3]) const noexcept {
        double xx = x*x, yy = y*y, zz = z*z;
        double xy = x*y, xz = x*z, yz = y*z;
        double wx = w*x, wy = w*y, wz = w*z;
        m[0][0] = 1.0 - 2.0*(yy+zz);
        m[0][1] = 2.0*(xy-wz);
        m[0][2] = 2.0*(xz+wy);
        m[1][0] = 2.0*(xy+wz);
        m[1][1] = 1.0 - 2.0*(xx+zz);
        m[1][2] = 2.0*(yz-wx);
        m[2][0] = 2.0*(xz-wy);
        m[2][1] = 2.0*(yz+wx);
        m[2][2] = 1.0 - 2.0*(xx+yy);
    }

    void normalize() noexcept {
        double n = norm();
        if (n < 1e-12) { w = 1.0; x = y = z = 0.0; return; }
        w /= n; x /= n; y /= n; z /= n;
    }

    [[nodiscard]] Quaternion conjugate() const noexcept {
        return Quaternion(w, -x, -y, -z);
    }

    [[nodiscard]] Quaternion inverse() const noexcept {
        double n2 = w*w + x*x + y*y + z*z;
        if (n2 < 1e-12) return Quaternion();
        Quaternion c = conjugate();
        return Quaternion(c.w / n2, c.x / n2, c.y / n2, c.z / n2);
    }

    [[nodiscard]] Quaternion operator*(const Quaternion& rhs) const noexcept {
        return Quaternion(
            w*rhs.w - x*rhs.x - y*rhs.y - z*rhs.z,
            w*rhs.x + x*rhs.w + y*rhs.z - z*rhs.y,
            w*rhs.y - x*rhs.z + y*rhs.w + z*rhs.x,
            w*rhs.z + x*rhs.y - y*rhs.x + z*rhs.w
        );
    }

    [[nodiscard]] Vector3 rotate(const Vector3& v) const noexcept {
        Quaternion qv(0.0, v);
        Quaternion res = (*this * qv * inverse());
        return Vector3(res.x, res.y, res.z);
    }

    [[nodiscard]] Vector3 forward() const noexcept { return rotate(Vector3(0,0,1)); }
    [[nodiscard]] Vector3 up() const noexcept { return rotate(Vector3(0,1,0)); }
    [[nodiscard]] Vector3 right() const noexcept { return rotate(Vector3(1,0,0)); }

    // Integrate angular velocity over dt, applying Magnus & effective gravity effects
    void integrateAngularVelocity(const Vector3& spin, double dt, const Vector3& velocity,
                                  double magnusCoeff, double gravityEffect) noexcept {
        angularVelocity = spin;
        Vector3 magnus = Vector3::magnusForce(spin, velocity, magnusCoeff); // FRC spin/ball Magnus
        Vector3 effectiveGravity = Vector3(0, 0, -9.81 * gravityEffect) + magnus; // dynamic gravity
        Vector3 deltaOmega = angularVelocity * dt; // simple Euler integration
        Quaternion deltaQuat(0.0, deltaOmega);
        *this = (*this + deltaQuat * (*this) * 0.5).normalized();
    }

    Quaternion operator+(const Quaternion& rhs) const noexcept {
        return Quaternion(w+rhs.w, x+rhs.x, y+rhs.y, z+rhs.z);
    }

    Quaternion operator*(double scalar) const noexcept {
        return Quaternion(w*scalar, x*scalar, y*scalar, z*scalar);
    }
    Quaternion operator-() const noexcept { return Quaternion(-w, -x, -y, -z); }

    [[nodiscard]] bool operator==(const Quaternion& rhs) const noexcept {
        return w==rhs.w && x==rhs.x && y==rhs.y && z==rhs.z;
    }

    [[nodiscard]] bool operator!=(const Quaternion& rhs) const noexcept {
        return !(*this == rhs);
    }

    friend std::ostream& operator<<(std::ostream& os, const Quaternion& q) {
        return os << "[" << q.w << ", (" << q.x << ", " << q.y << ", " << q.z << ")]";
    }
};

} // namespace frcsim