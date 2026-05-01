#pragma once

#include <cmath>

#include "frcsim/math/vector.hpp"
#include "frcsim/math/matrix.hpp"
#include "frcsim/math/quaternion.hpp"
#include "frcsim/aerodynamics/magnus_model.hpp"

namespace frcsim::aerodynamics {

/**
 * @brief Spin evolution model using existing Magnus + Drag ecosystem
 *
 * Responsibilities:
 *   - Angular damping (linear + nonlinear)
 *   - Magnus torque coupling via MagnusModel
 *
 * DOES NOT:
 *   - recompute drag or magnus physics manually
 */
class SpinDecayModel {
 public:
  /**
   * @brief Constructs a spin decay model.
   * Marked explicit to satisfy wpiformat rules and avoid implicit conversions.
   */
  explicit SpinDecayModel(const MagnusModel& magnus_model = MagnusModel())
      : magnus_model_(magnus_model) {}

  /**
   * @brief Advance angular velocity in time
   *
   * @param omegaLocal angular velocity in body frame
   * @param velocityWorld linear velocity in world frame
   * @param orientation body orientation quaternion
   * @param dt timestep
   */
  Vector3 step(const Vector3& omegaLocal,
               const Vector3& velocityWorld,
               const Quaternion& orientation,
               double dt) const noexcept {
    // --- Rotation transforms ---
    Matrix3 R = Matrix3::fromQuaternion(orientation);
    Matrix3 Rinv = R.transpose();

    // body -> world angular velocity
    Vector3 omegaWorld = R * omegaLocal;

    double vMag = velocityWorld.norm();
    double wMag = omegaWorld.norm();

    // --- Angular damping terms ---
    Vector3 linearDamping = omegaWorld * m_linearDecay;

    Vector3 velocityDamping = omegaWorld * (m_velocityCoupling * vMag);

    Vector3 nonlinearDamping = omegaWorld * (m_nonlinearDecay * wMag);

    // --- Magnus coupling (external model, no duplication) ---
    Vector3 magnusForce =
        magnus_model_.computeForce(velocityWorld, omegaWorld);

    // torque = r × F
    Vector3 magnusTorque = m_radiusVector.cross(magnusForce);

    // --- Angular acceleration ---
    Vector3 domega =
        magnusTorque
        - linearDamping
        - velocityDamping
        - nonlinearDamping;

    // integrate in world frame
    omegaWorld = omegaWorld + domega * dt;

    // world -> body
    return Rinv * omegaWorld;
  }

  // --- Tunables ---

  void setLinearDecay(double k) noexcept { m_linearDecay = k; }

  void setVelocityCoupling(double k) noexcept { m_velocityCoupling = k; }

  void setNonlinearDecay(double k) noexcept { m_nonlinearDecay = k; }

  void setRadiusVector(const Vector3& r) noexcept { m_radiusVector = r; }

 private:
  MagnusModel magnus_model_;

  double m_linearDecay{0.05};
  double m_velocityCoupling{0.01};
  double m_nonlinearDecay{0.02};

  // lever arm for torque (body-space assumption)
  Vector3 m_radiusVector{0.0, 0.0, 0.05};
};

}  // namespace frcsim::aerodynamics