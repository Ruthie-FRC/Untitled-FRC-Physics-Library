// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#include "frcsim/physics_world.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

#include "frcsim/aerodynamics/drag_model.hpp"

namespace frcsim {

RigidBody& PhysicsWorld::createBody(double mass_kg) {
  bodies_.emplace_back(mass_kg);
  if (!config_.enable_gravity) {
    bodies_.back().flags().enable_gravity = false;
  }
  return bodies_.back();
}

RigidAssembly& PhysicsWorld::createAssembly() {
  assemblies_.emplace_back();
  return assemblies_.back();
}

BallPhysicsSim3D& PhysicsWorld::createBall(
    const BallPhysicsSim3D::Config& config,
    const BallPhysicsSim3D::BallProperties& properties) {
  balls_.emplace_back(config, properties);
  return balls_.back();
}

EnvironmentalBoundary& PhysicsWorld::addBoundary() {
  boundaries_.emplace_back();
  return boundaries_.back();
}

void PhysicsWorld::addGlobalForceGenerator(
    const std::shared_ptr<ForceGenerator>& generator) {
  if (generator) {
    global_force_generators_.push_back(generator);
  }
}

void PhysicsWorld::step() {
  const double dt_s = config_.fixed_dt_s;

  auto resolve_boundary_contact = [&](RigidBody& body) {
    if (!config_.enable_collision_detection || body.flags().is_kinematic) {
      return;
    }

    const Material* body_material = body.material();
    const double body_restitution =
        body_material ? body_material->coefficient_of_restitution : 0.4;
    const double body_mu_kinetic =
        body_material ? body_material->coefficient_of_friction_kinetic : 0.6;

    for (const auto& boundary : boundaries_) {
      if (!boundary.is_active) {
        continue;
      }

      Vector3 contact_normal_world = boundary.orientation.rotate(boundary.normal());
      if (contact_normal_world.isZero()) {
        contact_normal_world = Vector3::unitZ();
      } else {
        contact_normal_world = contact_normal_world.normalized();
      }

      bool has_contact = false;
      double penetration_m = 0.0;

      if (boundary.type == BoundaryType::kPlane || boundary.type == BoundaryType::kWall) {
        const Vector3 rel = body.position() - boundary.position_m;
        const double signed_distance_m = rel.dot(contact_normal_world);
        if (signed_distance_m < 0.0) {
          has_contact = true;
          penetration_m = -signed_distance_m;
        }
      } else if (boundary.type == BoundaryType::kBox) {
        const Vector3 rel_world = body.position() - boundary.position_m;
        const Vector3 rel_local = boundary.orientation.inverse().rotate(rel_world);
        const Vector3 half = boundary.half_extents_m;

        const bool inside_xy =
            std::abs(rel_local.x) <= std::max(0.0, half.x) &&
            std::abs(rel_local.y) <= std::max(0.0, half.y);
        const double top_local_z = std::max(0.0, half.z);
        if (inside_xy && rel_local.z < top_local_z) {
          has_contact = true;
          penetration_m = top_local_z - rel_local.z;
          contact_normal_world = boundary.orientation.rotate(Vector3::unitZ()).normalized();
        }
      } else if (boundary.type == BoundaryType::kCylinder) {
        const Vector3 rel_world = body.position() - boundary.position_m;
        const Vector3 rel_local = boundary.orientation.inverse().rotate(rel_world);
        const double radial_m = std::sqrt(rel_local.x * rel_local.x + rel_local.y * rel_local.y);
        const double radius_m = std::max(0.0, boundary.radius_m);
        const double top_local_z = std::max(0.0, boundary.half_extents_m.z);

        if (radial_m <= radius_m && rel_local.z < top_local_z) {
          has_contact = true;
          penetration_m = top_local_z - rel_local.z;
          contact_normal_world = boundary.orientation.rotate(Vector3::unitZ()).normalized();
        }
      }

      if (!has_contact || penetration_m <= 0.0) {
        continue;
      }

      body.setPosition(body.position() + contact_normal_world * penetration_m);

      const Vector3 velocity = body.linearVelocity();
      const double vn = velocity.dot(contact_normal_world);
      if (vn >= 0.0) {
        continue;
      }

      const double restitution = std::clamp(
          0.5 * (boundary.restitution + body_restitution), 0.0, 1.0);
      const double friction =
          std::max(0.0, 0.5 * (boundary.friction_coefficient + body_mu_kinetic));

      const Vector3 v_normal = contact_normal_world * vn;
      const Vector3 v_tangent = velocity - v_normal;
      const Vector3 v_normal_after = contact_normal_world * (-vn * restitution);

      Vector3 v_tangent_after = v_tangent;
      const double vt_mag = v_tangent.norm();
      if (vt_mag > 1e-9) {
        const double friction_impulse_ratio =
            std::clamp(friction * (1.0 + restitution) * std::abs(vn) / vt_mag,
                       0.0, 1.0);
        v_tangent_after = v_tangent * (1.0 - friction_impulse_ratio);
      }

      body.setLinearVelocity(v_normal_after + v_tangent_after);
    }
  };

  auto step_body = [&](RigidBody& body) {
    if (config_.enable_aerodynamics && !body.flags().is_kinematic) {
      DragModel drag_model(config_.default_drag_coefficient,
                           config_.default_drag_reference_area_m2,
                           config_.air_density_kgpm3,
                           config_.linear_drag_coefficient_n_per_mps);
      body.applyForce(drag_model.computeForce(body));

      const Vector3 magnus =
          Vector3::magnusForce(body.linearVelocity(), body.angularVelocity(),
                               config_.magnus_coefficient);
      body.applyForce(magnus);
    }

    for (const auto& generator : global_force_generators_) {
      generator->apply(body, dt_s);
    }

    body.integrate(dt_s, config_.integration_method, config_.gravity_mps2,
                   config_.linear_damping_per_s, config_.angular_damping_per_s);

    resolve_boundary_contact(body);
  };

  for (auto& body : bodies_) {
    step_body(body);
  }

  for (auto& assembly : assemblies_) {
    for (auto& body : assembly.bodies()) {
      step_body(body);
    }
    if (config_.enable_joint_constraints) {
      assembly.solveConstraints(dt_s, 4);
    }
  }

  for (auto& ball : balls_) {
    ball.step(dt_s);
  }

  ++step_count_;
  accumulated_sim_time_s_ += dt_s;
}

}  // namespace frcsim
