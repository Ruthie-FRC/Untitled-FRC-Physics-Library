// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#include "frcsim/physics_world.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

#include "frcsim/aerodynamics/drag_model.hpp"

namespace frcsim {

/**
 * @brief Creates and adds a new RigidBody to the PhysicsWorld.
 *
 * @param mass_kg The mass of the body in kilograms.
 * @return Reference to the newly created RigidBody.
 */
RigidBody& PhysicsWorld::createBody(double mass_kg) {
  bodies_.emplace_back(mass_kg);
  if (!config_.enable_gravity) {
    bodies_.back().flags().enable_gravity = false;
  }
  return bodies_.back();
}

/**
 * @brief Creates and adds a new RigidAssembly to the PhysicsWorld.
 *
 * @return Reference to the newly created RigidAssembly.
 */
RigidAssembly& PhysicsWorld::createAssembly() {
  assemblies_.emplace_back();
  return assemblies_.back();
}

/**
 * @brief Creates and adds a new BallPhysicsSim3D to the PhysicsWorld.
 *
 * @param config Configuration for the ball simulation.
 * @param properties Physical properties of the ball.
 * @return Reference to the newly created BallPhysicsSim3D.
 */
BallPhysicsSim3D& PhysicsWorld::createBall(
    const BallPhysicsSim3D::Config& config,
    const BallPhysicsSim3D::BallProperties& properties) {
  balls_.emplace_back(config, properties);
  return balls_.back();
}

/**
 * @brief Adds a new EnvironmentalBoundary to the PhysicsWorld.
 *
 * @return Reference to the newly added EnvironmentalBoundary.
 */
EnvironmentalBoundary& PhysicsWorld::addBoundary() {
  boundaries_.emplace_back();
  return boundaries_.back();
}

/**
 * @brief Adds a global force generator to the PhysicsWorld.
 *
 * @param generator Shared pointer to the force generator to add.
 */
void PhysicsWorld::addGlobalForceGenerator(
    const std::shared_ptr<ForceGenerator>& generator) {
  if (generator) {
    global_force_generators_.push_back(generator);
  }
}

/**
 * @brief Sets the material interaction properties between two materials.
 *
 * @param interaction The MaterialInteraction to set.
 */
void PhysicsWorld::setMaterialInteraction(
    const MaterialInteraction& interaction) {
  const std::int32_t a = std::min(interaction.material_a_id,
                                  interaction.material_b_id);
  const std::int32_t b = std::max(interaction.material_a_id,
                                  interaction.material_b_id);
  for (auto& existing : material_interactions_) {
    const std::int32_t ea = std::min(existing.material_a_id,
                                     existing.material_b_id);
    const std::int32_t eb = std::max(existing.material_a_id,
                                     existing.material_b_id);
    if (ea == a && eb == b) {
      existing = interaction;
      return;
    }
  }
  material_interactions_.push_back(interaction);
}

void PhysicsWorld::clearMaterialInteractions() {
  material_interactions_.clear();
}

bool PhysicsWorld::shouldInteract(std::uint32_t layer_a, std::uint32_t mask_a,
                                  std::uint32_t layer_b,
                                  std::uint32_t mask_b) const {
  return ((layer_a & mask_b) != 0u) && ((layer_b & mask_a) != 0u);
}

const PhysicsWorld::MaterialInteraction* PhysicsWorld::findMaterialInteraction(
    std::int32_t material_a_id, std::int32_t material_b_id) const {
  const std::int32_t a = std::min(material_a_id, material_b_id);
  const std::int32_t b = std::max(material_a_id, material_b_id);
  for (const auto& interaction : material_interactions_) {
    if (!interaction.enabled) {
      continue;
    }
    const std::int32_t ia =
        std::min(interaction.material_a_id, interaction.material_b_id);
    const std::int32_t ib =
        std::max(interaction.material_a_id, interaction.material_b_id);
    if (ia == a && ib == b) {
      return &interaction;
    }
  }
  return nullptr;
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

      if (!shouldInteract(body.collisionLayer(), body.collisionMask(),
                          boundary.collision_layer_bits,
                          boundary.collision_mask_bits)) {
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

        double restitution = std::clamp(
          0.5 * (boundary.restitution + body_restitution), 0.0, 1.0);
        double friction =
          std::max(0.0, 0.5 * (boundary.friction_coefficient + body_mu_kinetic));

        if (const auto* pair =
            findMaterialInteraction(body.materialId(), boundary.material_id)) {
        restitution = std::clamp(pair->restitution, 0.0, 1.0);
        friction = std::max(0.0, pair->friction);
        }

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
