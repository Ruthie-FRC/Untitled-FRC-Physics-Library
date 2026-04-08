// Copyright (c) RenSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

#include "frcsim/math/vector.hpp"
#include "frcsim/rigidbody/rigid_body.hpp"

namespace frcsim {

struct DragGravityComparison {
  Vector3 drag_force{};
  double drag_force_magnitude_n{0.0};
  double drag_acceleration_mps2{0.0};
  double effective_gravity_acceleration_mps2{0.0};
  double drag_to_gravity_ratio{0.0};
  double body_mass_kg{0.0};
  bool valid{false};
};

class DragModel {
 public:
  DragModel(double drag_coefficient, double reference_area_m2,
            double air_density_kgpm3 = 1.225,
            double linear_drag_coefficient_n_per_mps = 0.0)
      : drag_coefficient_(drag_coefficient),
        reference_area_m2_(reference_area_m2),
        air_density_kgpm3_(air_density_kgpm3),
        linear_drag_coefficient_n_per_mps_(linear_drag_coefficient_n_per_mps) {}

  Vector3::DragForceDetails computeForceDetailed(
      const Vector3& velocity_mps) const {
    return Vector3::dragForceDetailed(velocity_mps, drag_coefficient_,
                                      reference_area_m2_, air_density_kgpm3_,
                                      linear_drag_coefficient_n_per_mps_);
  }

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

  Vector3 computeForce(const Vector3& velocity_mps) const {
    auto details = computeForceDetailed(velocity_mps);
    return Vector3(details.force.x, details.force.y, details.force.z);
  }

  Vector3 computeForce(const RigidBody& body) const {
    auto details = computeForceDetailed(body);
    return Vector3(details.force.x, details.force.y, details.force.z);
  }

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

  void apply(RigidBody& body) const {
    if (body.isStatic())
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
