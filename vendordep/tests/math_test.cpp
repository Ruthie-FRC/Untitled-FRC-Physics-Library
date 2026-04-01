#include <cassert>
#include <cmath>
#include <iostream>

#include "frcsim/math/quaternion.hpp"
#include "frcsim/math/vector.hpp"
#include "frcsim/math/matrix.hpp"

int main() {
    std::cout << "Testing math library...\n";

    // ===== Vector3 Tests =====
    {
        // Basic construction and zero
        frcsim::Vector3 v1;
        assert(v1.x == 0.0 && v1.y == 0.0 && v1.z == 0.0);

        frcsim::Vector3 v2(1.0, 2.0, 3.0);
        assert(v2.x == 1.0 && v2.y == 2.0 && v2.z == 3.0);

        // Addition
        frcsim::Vector3 v3 = v2 + v2;
        assert(v3.x == 2.0 && v3.y == 4.0 && v3.z == 6.0);

        // Subtraction
        frcsim::Vector3 v4 = v3 - v2;
        assert(v4.x == 1.0 && v4.y == 2.0 && v4.z == 3.0);

        // Scalar multiplication
        frcsim::Vector3 v5 = v2 * 2.0;
        assert(v5.x == 2.0 && v5.y == 4.0 && v5.z == 6.0);

        // Dot product
        double dot = v2.dot(v2);
        assert(std::fabs(dot - 14.0) < 1e-9);  // 1^2 + 2^2 + 3^2 = 14

        // Cross product
        frcsim::Vector3 i(1.0, 0.0, 0.0);
        frcsim::Vector3 j(0.0, 1.0, 0.0);
        frcsim::Vector3 cross = i.cross(j);
        assert(std::fabs(cross.x - 0.0) < 1e-9);
        assert(std::fabs(cross.y - 0.0) < 1e-9);
        assert(std::fabs(cross.z - 1.0) < 1e-9);

        // Magnitude
        double mag = v2.magnitude();
        assert(std::fabs(mag - std::sqrt(14.0)) < 1e-9);

        // Normalized
        frcsim::Vector3 normalized = v2.normalized();
        double normalized_mag = normalized.magnitude();
        assert(std::fabs(normalized_mag - 1.0) < 1e-9);
    }

    // ===== Quaternion Tests =====
    {
        // Identity quaternion
        frcsim::Quaternion q_identity;
        assert(q_identity.w == 1.0);
        assert(q_identity.x == 0.0 && q_identity.y == 0.0 && q_identity.z == 0.0);

        // Quaternion multiplication
        frcsim::Quaternion q1(1.0, 0.0, 0.0, 0.0);  // identity
        frcsim::Quaternion q2(0.7071, 0.7071, 0.0, 0.0);  // 90 deg rotation around X
        frcsim::Quaternion q3 = q1 * q2;
        assert(std::fabs(q3.w - q2.w) < 1e-3);
        assert(std::fabs(q3.x - q2.x) < 1e-3);

        // Normalization
        frcsim::Quaternion q_unnormalized(1.0, 1.0, 1.0, 1.0);
        q_unnormalized.normalizeIfNeeded();
        double mag_sq = q_unnormalized.w * q_unnormalized.w +
                        q_unnormalized.x * q_unnormalized.x +
                        q_unnormalized.y * q_unnormalized.y +
                        q_unnormalized.z * q_unnormalized.z;
        assert(std::fabs(mag_sq - 1.0) < 1e-9);

        // Conjugate
        frcsim::Quaternion q_conj = q2.conjugate();
        assert(std::fabs(q_conj.w - q2.w) < 1e-9);
        assert(std::fabs(q_conj.x + q2.x) < 1e-9);

        // Rotate vector
        frcsim::Vector3 v_original(1.0, 0.0, 0.0);
        frcsim::Vector3 v_rotated = q2.rotateVector(v_original);
        // 90-degree rotation around X should map (1,0,0) -> close to (1,0,0)
        assert(std::fabs(v_rotated.x - 1.0) < 1e-3);
    }

    // ===== Matrix3 Tests =====
    {
        // Identity matrix
        frcsim::Matrix3 m_identity;
        assert(m_identity.m[0][0] == 1.0 && m_identity.m[1][1] == 1.0 && m_identity.m[2][2] == 1.0);

        // Matrix-vector multiplication
        frcsim::Vector3 v(1.0, 2.0, 3.0);
        frcsim::Vector3 result = m_identity * v;
        assert(std::fabs(result.x - 1.0) < 1e-9);
        assert(std::fabs(result.y - 2.0) < 1e-9);
        assert(std::fabs(result.z - 3.0) < 1e-9);

        // Determinant of identity
        double det = m_identity.determinant();
        assert(std::fabs(det - 1.0) < 1e-9);

        // Transpose of identity
        frcsim::Matrix3 m_transpose = m_identity.transpose();
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                assert(std::fabs(m_transpose.m[i][j] - m_identity.m[j][i]) < 1e-9);
            }
        }
    }

    std::cout << "✓ All math tests passed!\n";
    return 0;
}
