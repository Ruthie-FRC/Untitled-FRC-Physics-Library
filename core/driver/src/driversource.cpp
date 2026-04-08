// Copyright (c) RenSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#include "driverheader.h"

#include <algorithm>
#include <cstdint>
#include <memory>
#include <mutex>
#include <unordered_map>

#include "frcsim/physics_world.hpp"

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
}  // extern "C"
