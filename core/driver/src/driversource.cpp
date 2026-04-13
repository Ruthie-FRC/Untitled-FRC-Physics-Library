// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#include "driverheader.h"

#include <algorithm>
#include <cstdint>
#include <memory>
#include <mutex>
#include <unordered_map>

#include "frcsim/physics_world.hpp"
#include "frcsim/rigidbody/material.hpp"

namespace {

std::mutex g_world_mutex;
std::unordered_map<std::uint64_t, std::unique_ptr<frcsim::PhysicsWorld>>
    g_worlds;
std::uint64_t g_next_handle = 1;

frcsim::PhysicsWorld* getWorld(std::uint64_t handle) {
  const auto it = g_worlds.find(handle);
  if (it == g_worlds.end()) {
    return nullptr;
  }
  return it->second.get();
}

frcsim::RigidBody* getBody(frcsim::PhysicsWorld* world, int body_index) {
  if (!world || body_index < 0) {
    return nullptr;
  }
  auto& bodies = world->bodies();
  const std::size_t idx = static_cast<std::size_t>(body_index);
  if (idx >= bodies.size()) {
    return nullptr;
  }
  return &bodies[idx];
}

}  // namespace

extern "C" {
void c_doThing(void) {}

uint64_t c_rsCreateWorld(double fixed_dt_s, int enable_gravity) {
  std::lock_guard<std::mutex> lock(g_world_mutex);
  frcsim::PhysicsConfig config;
  config.fixed_dt_s = (fixed_dt_s > 0.0) ? fixed_dt_s : 0.01;
  config.enable_gravity = (enable_gravity != 0);

  const std::uint64_t handle = g_next_handle++;
  g_worlds.emplace(handle, std::make_unique<frcsim::PhysicsWorld>(config));
  return handle;
}

void c_rsDestroyWorld(uint64_t world_handle) {
  std::lock_guard<std::mutex> lock(g_world_mutex);
  g_worlds.erase(world_handle);
}

int c_rsCreateBody(uint64_t world_handle, double mass_kg) {
  std::lock_guard<std::mutex> lock(g_world_mutex);
  frcsim::PhysicsWorld* world = getWorld(world_handle);
  if (!world) {
    return -1;
  }

  world->createBody(mass_kg);
  return static_cast<int>(world->bodies().size() - 1);
}

int c_rsSetBodyPosition(uint64_t world_handle, int body_index,
                        double x_m, double y_m, double z_m) {
  std::lock_guard<std::mutex> lock(g_world_mutex);
  frcsim::PhysicsWorld* world = getWorld(world_handle);
  frcsim::RigidBody* body = getBody(world, body_index);
  if (!body) {
    return -1;
  }
  body->setPosition(frcsim::Vector3{x_m, y_m, z_m});
  return 0;
}

int c_rsSetBodyLinearVelocity(uint64_t world_handle, int body_index,
                              double vx_mps, double vy_mps, double vz_mps) {
  std::lock_guard<std::mutex> lock(g_world_mutex);
  frcsim::PhysicsWorld* world = getWorld(world_handle);
  frcsim::RigidBody* body = getBody(world, body_index);
  if (!body) {
    return -1;
  }
  body->setLinearVelocity(frcsim::Vector3{vx_mps, vy_mps, vz_mps});
  return 0;
}

int c_rsSetBodyGravityEnabled(uint64_t world_handle, int body_index,
                              int enabled) {
  std::lock_guard<std::mutex> lock(g_world_mutex);
  frcsim::PhysicsWorld* world = getWorld(world_handle);
  frcsim::RigidBody* body = getBody(world, body_index);
  if (!body) {
    return -1;
  }
  body->flags().enable_gravity = (enabled != 0);
  return 0;
}

int c_rsSetBodyMaterial(uint64_t world_handle, int body_index,
                        double restitution, double friction_kinetic,
                        double friction_static, double collision_damping) {
  std::lock_guard<std::mutex> lock(g_world_mutex);
  frcsim::PhysicsWorld* world = getWorld(world_handle);
  frcsim::RigidBody* body = getBody(world, body_index);
  if (!body) {
    return -1;
  }

  frcsim::Material material;
  material.coefficient_of_restitution =
      std::clamp(restitution, 0.0, 1.0);
  material.coefficient_of_friction_kinetic =
      std::max(0.0, friction_kinetic);
  material.coefficient_of_friction_static =
      std::max(0.0, friction_static);
  material.collision_damping = std::clamp(collision_damping, 0.0, 1.0);

  body->setMaterial(material);
  return 0;
}

int c_rsSetBodyMaterialId(uint64_t world_handle, int body_index,
                          int32_t material_id) {
  std::lock_guard<std::mutex> lock(g_world_mutex);
  frcsim::PhysicsWorld* world = getWorld(world_handle);
  frcsim::RigidBody* body = getBody(world, body_index);
  if (!body) {
    return -1;
  }

  body->setMaterialId(material_id);
  return 0;
}

int c_rsSetBodyCollisionFilter(uint64_t world_handle, int body_index,
                               uint32_t collision_layer_bits,
                               uint32_t collision_mask_bits) {
  std::lock_guard<std::mutex> lock(g_world_mutex);
  frcsim::PhysicsWorld* world = getWorld(world_handle);
  frcsim::RigidBody* body = getBody(world, body_index);
  if (!body) {
    return -1;
  }

  body->setCollisionLayer(collision_layer_bits);
  body->setCollisionMask(collision_mask_bits);
  return 0;
}

int c_rsSetBodyAerodynamicSphere(uint64_t world_handle, int body_index,
                                 double radius_m, double drag_coefficient) {
  std::lock_guard<std::mutex> lock(g_world_mutex);
  frcsim::PhysicsWorld* world = getWorld(world_handle);
  frcsim::RigidBody* body = getBody(world, body_index);
  if (!body) {
    return -1;
  }

  frcsim::RigidBody::AerodynamicGeometry geometry;
  geometry.shape = frcsim::RigidBody::AerodynamicGeometry::Shape::kSphere;
  geometry.radius_m = std::max(0.0, radius_m);
  body->setAerodynamicGeometry(geometry);

  world->config().default_drag_coefficient =
      std::max(0.0, drag_coefficient);
  return 0;
}

int c_rsSetBodyAerodynamicBox(uint64_t world_handle, int body_index,
                              double x_m, double y_m, double z_m,
                              double drag_coefficient) {
  std::lock_guard<std::mutex> lock(g_world_mutex);
  frcsim::PhysicsWorld* world = getWorld(world_handle);
  frcsim::RigidBody* body = getBody(world, body_index);
  if (!body) {
    return -1;
  }

  frcsim::RigidBody::AerodynamicGeometry geometry;
  geometry.shape = frcsim::RigidBody::AerodynamicGeometry::Shape::kBox;
  geometry.box_dimensions_m =
      frcsim::Vector3(std::max(0.0, x_m), std::max(0.0, y_m), std::max(0.0, z_m));
  body->setAerodynamicGeometry(geometry);

  world->config().default_drag_coefficient =
      std::max(0.0, drag_coefficient);
  return 0;
}

int c_rsSetWorldAerodynamics(uint64_t world_handle, int enabled,
                             double air_density_kgpm3,
                             double linear_drag_coefficient_n_per_mps,
                             double magnus_coefficient,
                             double default_drag_coefficient,
                             double default_drag_reference_area_m2) {
  std::lock_guard<std::mutex> lock(g_world_mutex);
  frcsim::PhysicsWorld* world = getWorld(world_handle);
  if (!world) {
    return -1;
  }

  auto& cfg = world->config();
  cfg.enable_aerodynamics = (enabled != 0);
  cfg.air_density_kgpm3 = std::max(0.0, air_density_kgpm3);
  cfg.linear_drag_coefficient_n_per_mps =
      std::max(0.0, linear_drag_coefficient_n_per_mps);
  cfg.magnus_coefficient = magnus_coefficient;
  cfg.default_drag_coefficient = std::max(0.0, default_drag_coefficient);
  cfg.default_drag_reference_area_m2 =
      std::max(0.0, default_drag_reference_area_m2);

  return 0;
}

int c_rsSetMaterialInteraction(uint64_t world_handle, int32_t material_a_id,
                               int32_t material_b_id, double restitution,
                               double friction, int enabled) {
  std::lock_guard<std::mutex> lock(g_world_mutex);
  frcsim::PhysicsWorld* world = getWorld(world_handle);
  if (!world) {
    return -1;
  }

  frcsim::PhysicsWorld::MaterialInteraction interaction;
  interaction.material_a_id = material_a_id;
  interaction.material_b_id = material_b_id;
  interaction.restitution = std::clamp(restitution, 0.0, 1.0);
  interaction.friction = std::max(0.0, friction);
  interaction.enabled = (enabled != 0);
  world->setMaterialInteraction(interaction);
  return 0;
}

int c_rsStepWorld(uint64_t world_handle, int steps) {
  std::lock_guard<std::mutex> lock(g_world_mutex);
  frcsim::PhysicsWorld* world = getWorld(world_handle);
  if (!world) {
    return -1;
  }

  const int safe_steps = std::max(steps, 1);
  for (int i = 0; i < safe_steps; ++i) {
    world->step();
  }
  return 0;
}

int c_rsSetWorldGravity(uint64_t world_handle, double gx_mps2,
                        double gy_mps2, double gz_mps2) {
  std::lock_guard<std::mutex> lock(g_world_mutex);
  frcsim::PhysicsWorld* world = getWorld(world_handle);
  if (!world) {
    return -1;
  }

  world->config().gravity_mps2 = frcsim::Vector3{gx_mps2, gy_mps2, gz_mps2};
  world->config().enable_gravity = true;
  return 0;
}

int c_rsGetBodyPosition(uint64_t world_handle, int body_index,
                        double* x_m, double* y_m, double* z_m) {
  if (!x_m || !y_m || !z_m) {
    return -1;
  }

  std::lock_guard<std::mutex> lock(g_world_mutex);
  frcsim::PhysicsWorld* world = getWorld(world_handle);
  frcsim::RigidBody* body = getBody(world, body_index);
  if (!body) {
    return -1;
  }

  const frcsim::Vector3 p = body->position();
  *x_m = p.x;
  *y_m = p.y;
  *z_m = p.z;
  return 0;
}

int c_rsGetBodyLinearVelocity(uint64_t world_handle, int body_index,
                              double* vx_mps, double* vy_mps, double* vz_mps) {
  if (!vx_mps || !vy_mps || !vz_mps) {
    return -1;
  }

  std::lock_guard<std::mutex> lock(g_world_mutex);
  frcsim::PhysicsWorld* world = getWorld(world_handle);
  frcsim::RigidBody* body = getBody(world, body_index);
  if (!body) {
    return -1;
  }

  const frcsim::Vector3 v = body->linearVelocity();
  *vx_mps = v.x;
  *vy_mps = v.y;
  *vz_mps = v.z;
  return 0;
}

int c_rsGetBodyPose7Array(uint64_t world_handle, double* out_pose7,
                          int max_bodies) {
  if (!out_pose7 || max_bodies < 0) {
    return -1;
  }

  std::lock_guard<std::mutex> lock(g_world_mutex);
  frcsim::PhysicsWorld* world = getWorld(world_handle);
  if (!world) {
    return -1;
  }

  const auto& bodies = world->bodies();
  const int count = std::min(max_bodies, static_cast<int>(bodies.size()));
  for (int i = 0; i < count; ++i) {
    const auto& body = bodies[static_cast<std::size_t>(i)];
    const auto p = body.position();
    const auto q = body.orientation();
    const int base = i * 7;
    out_pose7[base + 0] = p.x;
    out_pose7[base + 1] = p.y;
    out_pose7[base + 2] = p.z;
    out_pose7[base + 3] = q.w;
    out_pose7[base + 4] = q.x;
    out_pose7[base + 5] = q.y;
    out_pose7[base + 6] = q.z;
  }
  return count;
}

int c_rsGetBodyVelocity6Array(uint64_t world_handle, double* out_velocity6,
                              int max_bodies) {
  if (!out_velocity6 || max_bodies < 0) {
    return -1;
  }

  std::lock_guard<std::mutex> lock(g_world_mutex);
  frcsim::PhysicsWorld* world = getWorld(world_handle);
  if (!world) {
    return -1;
  }

  const auto& bodies = world->bodies();
  const int count = std::min(max_bodies, static_cast<int>(bodies.size()));
  for (int i = 0; i < count; ++i) {
    const auto& body = bodies[static_cast<std::size_t>(i)];
    const auto v = body.linearVelocity();
    const auto w = body.angularVelocity();
    const int base = i * 6;
    out_velocity6[base + 0] = v.x;
    out_velocity6[base + 1] = v.y;
    out_velocity6[base + 2] = v.z;
    out_velocity6[base + 3] = w.x;
    out_velocity6[base + 4] = w.y;
    out_velocity6[base + 5] = w.z;
  }
  return count;
}

int c_rsGetBodyState13Array(uint64_t world_handle, double* out_state13,
                            int max_bodies) {
  if (!out_state13 || max_bodies < 0) {
    return -1;
  }

  std::lock_guard<std::mutex> lock(g_world_mutex);
  frcsim::PhysicsWorld* world = getWorld(world_handle);
  if (!world) {
    return -1;
  }

  const auto& bodies = world->bodies();
  const int count = std::min(max_bodies, static_cast<int>(bodies.size()));
  for (int i = 0; i < count; ++i) {
    const auto& body = bodies[static_cast<std::size_t>(i)];
    const auto p = body.position();
    const auto q = body.orientation();
    const auto v = body.linearVelocity();
    const auto w = body.angularVelocity();

    const int base = i * 13;
    out_state13[base + 0] = p.x;
    out_state13[base + 1] = p.y;
    out_state13[base + 2] = p.z;
    out_state13[base + 3] = q.w;
    out_state13[base + 4] = q.x;
    out_state13[base + 5] = q.y;
    out_state13[base + 6] = q.z;
    out_state13[base + 7] = v.x;
    out_state13[base + 8] = v.y;
    out_state13[base + 9] = v.z;
    out_state13[base + 10] = w.x;
    out_state13[base + 11] = w.y;
    out_state13[base + 12] = w.z;
  }
  return count;
}
}  // extern "C"
