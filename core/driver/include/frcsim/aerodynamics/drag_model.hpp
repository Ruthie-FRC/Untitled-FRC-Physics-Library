// Copyright (c) RenSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

#include "frcsim/math/vector.hpp"
#include "frcsim/rigidbody/rigid_body.hpp"

namespace frcsim {

/**
 * @brief Diagnostic output comparing drag force to gravitational acceleration.
 */
struct DragGravityComparison {
  /** @brief Computed drag force vector in newtons. */
  Vector3 drag_force{};
  /** @brief Magnitude of drag_force in newtons. */
  double drag_force_magnitude_n{0.0};
  /** @brief Drag-induced acceleration magnitude in m/s^2. */
  double drag_acceleration_mps2{0.0};
  /** @brief Magnitude of effective gravity vector in m/s^2. */
  double effective_gravity_acceleration_mps2{0.0};
  /** @brief Ratio of drag acceleration to gravity (>1 means drag dominates). */
  double drag_to_gravity_ratio{0.0};
  /** @brief Body mass used for acceleration calculations in kg. */
  double body_mass_kg{0.0};
  /** @brief True when the comparison computed without invalid inputs or mass=0. */
  bool valid{false};
};

/**
 * @brief Evaluates drag force on a moving body using quadratic and linear drag models.
 *
 * The drag force is computed as quad + linear from Vector3::dragForce().
 * Useful for validating aerodynamic behavior and comparing against gravity.
 */
class DragModel {
 public:
  /**
   * @brief Constructs a drag force model.
   * @param drag_coefficient Dimensionless drag coefficient (typically 0.1–2.0).
   * @param reference_area_m2 Reference area in square meters (cross-section).
   * @param air_density_kgpm3 Air density in kg/m^3 (default: 1.225 at sea level).
   * @param linear_drag_coefficient_n_per_mps Optional linear drag term in N/(m/s).
   */
  DragModel(double drag_coefficient, double reference_area_m2,
            double air_density_kgpm3 = 1.225,
            double linear_drag_coefficient_n_per_mps = 0.0)
      : drag_coefficient_(drag_coefficient),
        reference_area_m2_(reference_area_m2),
        air_density_kgpm3_(air_density_kgpm3),
        linear_drag_coefficient_n_per_mps_(linear_drag_coefficient_n_per_mps) {}

  /**
     * @brief Computes detailed drag force breakdown for a world-space velocity vector.
     * @param velocity_mps World-space velocity in m/s.
   * @return DragForceDetails with force, direction, and diagnostic fields.
   */
  Vector3::DragForceDetails computeForceDetailed(
      const Vector3& velocity_mps) const {
    return Vector3::dragForceDetailed(velocity_mps, drag_coefficient_,
                                      reference_area_m2_, air_density_kgpm3_,
                                      linear_drag_coefficient_n_per_mps_);
  }

  /**
    * @brief Computes detailed drag force using body geometry and motion.
    * @param body Rigid body supplying world-space velocity and drag area geometry.
   * @return DragForceDetails with force, direction, and diagnostics.
   */
  Vector3::DragForceDetails computeForceDetailed(const RigidBody& body) const {
    const Vector3& velocity_mps = body.linearVelocity();
    double reference_area_m2 = body.dragReferenceAreaM2(velocity_mps);
    if (reference_area_m2 <= 0.0) {
      reference_area_m2 = reference_area_m2_;
    }

    return Vector3::dragForceDetailed(velocity_mps, drag_coefficient_,
                                      reference_area_m2, air_density_kgpm3_,
                                      linear_drag_coefficient_n_per_mps_);
  }

  /**
    * @brief Computes drag force vector for a world-space velocity.
    * @param velocity_mps World-space velocity in m/s.
   * @return Drag force vector in newtons.
   */
  Vector3 computeForce(const Vector3& velocity_mps) const {
    auto details = computeForceDetailed(velocity_mps);
    return Vector3(details.force.x, details.force.y, details.force.z);
  }

  /**
   * @brief Computes drag force using body geometry.
    * @param body Rigid body to evaluate drag for using its world-space velocity.
   * @return Drag force vector in newtons.
   */
  Vector3 computeForce(const RigidBody& body) const {
    auto details = computeForceDetailed(body);
    return Vector3(details.force.x, details.force.y, details.force.z);
  }

  /**
   * @brief Compares drag force magnitude against an effective gravity vector.
   * @param body Rigid body to evaluate.
    * @param effective_gravity_mps2 Effective gravity vector in m/s^2 (e.g., gravity + Magnus).
   * @return Diagnostic comparison showing drag-to-gravity ratio.
   */
  DragGravityComparison compareToEffectiveGravity(
      const RigidBody& body, const Vector3& effective_gravity_mps2) const {
    DragGravityComparison comparison{};
    comparison.body_mass_kg = body.massKg();

    const auto details = computeForceDetailed(body);
    comparison.drag_force =
        Vector3(details.force.x, details.force.y, details.force.z);
    comparison.drag_force_magnitude_n = details.drag_force_magnitude_n;
    comparison.drag_acceleration_mps2 =
        (comparison.body_mass_kg > 0.0)
            ? comparison.drag_force_magnitude_n / comparison.body_mass_kg
            : 0.0;
    comparison.effective_gravity_acceleration_mps2 =
        effective_gravity_mps2.norm();
    comparison.drag_to_gravity_ratio =
        (comparison.effective_gravity_acceleration_mps2 > 0.0)
            ? comparison.drag_acceleration_mps2 /
                  comparison.effective_gravity_acceleration_mps2
            : 0.0;
    comparison.valid = details.valid && comparison.body_mass_kg > 0.0;
    return comparison;
  }

  /**
   * @brief Applies computed drag force to a rigid body.
   * @param body Rigid body to apply drag force to; kinematic bodies are skipped.
   */
  void apply(RigidBody& body) const {
    if (body.flags().is_kinematic)
      return;
    body.applyForce(computeForce(body.linearVelocity()));
  }

 private:
  double drag_coefficient_{0.47};
  double reference_area_m2_{0.02};
  double air_density_kgpm3_{1.225};
  double linear_drag_coefficient_n_per_mps_{0.0};
};

}  // namespace frcsim
