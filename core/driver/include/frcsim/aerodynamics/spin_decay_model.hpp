#pragma once

#include <cmath>

#include "frcsim/math/vector.hpp"
#include "frcsim/math/matrix.hpp"
#include "frcsim/math/quaternion.hpp"
#include "frcsim/aerodynamics/magnus_model.hpp"

namespace frcsim::aerodynamics {

/**
 * @file spin_decay_model.hpp
 * @brief Models angular velocity evolution with damping and Magnus coupling.
 *
 * This class integrates rotational dynamics for a spinning object using:
 * - Linear angular damping
 * - Velocity-coupled damping
 * - Nonlinear (angular magnitude) damping
 * - Magnus-induced torque via an external MagnusModel
 *
 * The model operates by transforming angular velocity into world space,
 * applying forces and torques, integrating forward in time, and transforming
 * back into body space.
 *
 * @note This class does NOT recompute drag or Magnus forces manually.
 *       It delegates Magnus force computation to MagnusModel.
 */
class SpinDecayModel {
 public:
  /**
   * @brief Constructs a spin decay model.
   *
   * @param magnus_model External Magnus force model used for torque coupling
   *
   * @note Explicit to prevent unintended implicit conversions.
   */
  explicit SpinDecayModel(const MagnusModel& magnus_model = MagnusModel())
      : magnus_model_(magnus_model) {}

  /**
   * @brief Advances angular velocity forward in time.
   *
   * Performs:
   * - Frame transformation (body → world)
   * - Damping calculations (linear, velocity-coupled, nonlinear)
   * - Magnus torque computation via MagnusModel
   * - Time integration in world frame
   * - Transformation back to body frame
   *
   * @param omegaLocal Angular velocity in body frame
   * @param velocityWorld Linear velocity in world frame
   * @param orientation Body orientation as quaternion
   * @param dt Time step (seconds)
   *
   * @return Updated angular velocity in body frame
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

    /** Linear damping proportional to angular velocity */
    Vector3 linearDamping = omegaWorld * m_linearDecay;

    /** Velocity-coupled damping scales with linear speed */
    Vector3 velocityDamping = omegaWorld * (m_velocityCoupling * vMag);

    /** Nonlinear damping scales with angular magnitude */
    Vector3 nonlinearDamping = omegaWorld * (m_nonlinearDecay * wMag);

    // --- Magnus coupling (external model) ---

    /**
     * Magnus force computed from external model.
     * This avoids duplicating aerodynamic logic.
     */
    Vector3 magnusForce =
        magnus_model_.computeForce(velocityWorld, omegaWorld);

    /** Torque generated via lever arm cross force */
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

  /**
   * @brief Sets linear angular decay coefficient.
   * @param k Linear damping coefficient
   */
  void setLinearDecay(double k) noexcept { m_linearDecay = k; }

  /**
   * @brief Sets velocity-coupled damping coefficient.
   * @param k Coupling coefficient with linear velocity magnitude
   */
  void setVelocityCoupling(double k) noexcept { m_velocityCoupling = k; }

  /**
   * @brief Sets nonlinear angular decay coefficient.
   * @param k Nonlinear damping coefficient (scales with angular magnitude)
   */
  void setNonlinearDecay(double k) noexcept { m_nonlinearDecay = k; }

  /**
   * @brief Sets radius (lever arm) used for Magnus torque calculation.
   *
   * @param r Radius vector in body frame
   */
  void setRadiusVector(const Vector3& r) noexcept { m_radiusVector = r; }

 private:
  /** External Magnus force model */
  MagnusModel magnus_model_;

  /** Linear angular damping coefficient */
  double m_linearDecay{0.05};

  /** Velocity-coupled damping coefficient */
  double m_velocityCoupling{0.01};

  /** Nonlinear angular damping coefficient */
  double m_nonlinearDecay{0.02};

  /**
   * @brief Lever arm used to compute torque (r × F).
   *
   * Assumed to be defined in body space.
   */
  Vector3 m_radiusVector{0.0, 0.0, 0.05};
};

}  // namespace frcsim::aerodynamics