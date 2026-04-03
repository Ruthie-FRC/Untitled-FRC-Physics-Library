#include "header.h"
#include "driverheader.h"

namespace rensim {

PhysicsWorld::PhysicsWorld(double fixed_dt_seconds, bool enable_gravity)
    : handle_(c_rsCreateWorld(fixed_dt_seconds, enable_gravity ? 1 : 0)) {}

PhysicsWorld::~PhysicsWorld() {
  if (handle_ != 0) {
    c_rsDestroyWorld(handle_);
    handle_ = 0;
  }
}

PhysicsWorld::PhysicsWorld(PhysicsWorld&& other) noexcept {
  handle_ = other.handle_;
  other.handle_ = 0;
}

PhysicsWorld& PhysicsWorld::operator=(PhysicsWorld&& other) noexcept {
  if (this == &other) {
    return *this;
  }

  if (handle_ != 0) {
    c_rsDestroyWorld(handle_);
  }
  handle_ = other.handle_;
  other.handle_ = 0;
  return *this;
}

int PhysicsWorld::createBody(double mass_kg) {
  if (handle_ == 0) {
    return -1;
  }
  return c_rsCreateBody(handle_, mass_kg);
}

bool PhysicsWorld::setBodyPosition(int body_index, const Vec3& position_m) {
  return handle_ != 0 &&
         c_rsSetBodyPosition(handle_, body_index, position_m.x, position_m.y, position_m.z) == 0;
}

bool PhysicsWorld::setBodyLinearVelocity(int body_index, const Vec3& velocity_mps) {
  return handle_ != 0 &&
         c_rsSetBodyLinearVelocity(handle_, body_index, velocity_mps.x, velocity_mps.y, velocity_mps.z) == 0;
}

bool PhysicsWorld::setBodyGravityEnabled(int body_index, bool enabled) {
  return handle_ != 0 && c_rsSetBodyGravityEnabled(handle_, body_index, enabled ? 1 : 0) == 0;
}

bool PhysicsWorld::setGravity(const Vec3& gravity_mps2) {
  return handle_ != 0 && c_rsSetWorldGravity(handle_, gravity_mps2.x, gravity_mps2.y, gravity_mps2.z) == 0;
}

bool PhysicsWorld::step(int steps) {
  return handle_ != 0 && c_rsStepWorld(handle_, steps) == 0;
}

bool PhysicsWorld::bodyPosition(int body_index, Vec3* out_position_m) const {
  if (handle_ == 0 || out_position_m == nullptr) {
    return false;
  }

  return c_rsGetBodyPosition(handle_, body_index, &out_position_m->x, &out_position_m->y, &out_position_m->z) == 0;
}

bool PhysicsWorld::bodyLinearVelocity(int body_index, Vec3* out_velocity_mps) const {
  if (handle_ == 0 || out_velocity_mps == nullptr) {
    return false;
  }

  return c_rsGetBodyLinearVelocity(handle_, body_index, &out_velocity_mps->x, &out_velocity_mps->y,
                                   &out_velocity_mps->z) == 0;
}

}  // namespace rensim
