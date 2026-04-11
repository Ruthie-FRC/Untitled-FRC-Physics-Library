// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#include "frcsim/rigidbody/rigid_assembly.hpp"

#include <memory>

#include "frcsim/joints/fixed_joint.hpp"
#include "frcsim/joints/prismatic_joint.hpp"
#include "frcsim/joints/revolute_joint.hpp"

namespace frcsim {

RigidBody* RigidAssembly::addBody(double mass_kg) {
  bodies_.emplace_back(mass_kg);
  return &bodies_.back();
}

RevoluteJoint* RigidAssembly::addRevoluteJoint(size_t body_a_idx,
                                               size_t body_b_idx,
                                               const Vector3& axis_local) {
  if (body_a_idx >= bodies_.size() || body_b_idx >= bodies_.size())
    return nullptr;
  auto joint = std::make_shared<RevoluteJoint>(
      &bodies_[body_a_idx], &bodies_[body_b_idx], axis_local);
  RevoluteJoint* ptr = joint.get();
  joints_.push_back(joint);
  return ptr;
}

PrismaticJoint* RigidAssembly::addPrismaticJoint(size_t body_a_idx,
                                                 size_t body_b_idx,
                                                 const Vector3& axis_local) {
  if (body_a_idx >= bodies_.size() || body_b_idx >= bodies_.size())
    return nullptr;
  auto joint = std::make_shared<PrismaticJoint>(
      &bodies_[body_a_idx], &bodies_[body_b_idx], axis_local);
  PrismaticJoint* ptr = joint.get();
  joints_.push_back(joint);
  return ptr;
}

FixedJoint* RigidAssembly::addFixedJoint(size_t body_a_idx, size_t body_b_idx) {
  if (body_a_idx >= bodies_.size() || body_b_idx >= bodies_.size())
    return nullptr;
  auto joint =
      std::make_shared<FixedJoint>(&bodies_[body_a_idx], &bodies_[body_b_idx]);
  FixedJoint* ptr = joint.get();
  joints_.push_back(joint);
  return ptr;
}

void RigidAssembly::setRootBodyIndex(size_t idx) {
  if (idx < bodies_.size()) {
    root_body_idx_ = idx;
  }
}

double RigidAssembly::totalMass() const {
  double total = 0.0;
  for (const auto& body : bodies_) {
    total += body.massKg();
  }
  return total;
}

Vector3 RigidAssembly::centerOfMass() const {
  Vector3 com{};
  double total_mass = 0.0;
  for (const auto& body : bodies_) {
    const double mass = body.massKg();
    if (mass > 0.0) {
      com += body.position() * mass;
      total_mass += mass;
    }
  }
  if (total_mass > 1e-12) {
    return com / total_mass;
  }
  return Vector3::zero();
}

void RigidAssembly::enableConstraints(bool enable) {
  for (auto& joint : joints_) {
    joint->setEnabled(enable);
  }
}

void RigidAssembly::solveConstraints(double dt_s, int iterations) {
  for (auto& joint : joints_) {
    if (joint->isEnabled()) {
      joint->solveConstraint(dt_s, iterations);
    }
  }
}

}  // namespace frcsim
