#pragma once

#include <algorithm>
#include <memory>
#include <units/mass.h>
#include <units/time.h>
#include "frcsim/config/physics_config.hpp"
#include "frcsim/math/integrators.hpp"
#include "frcsim/math/matrix.hpp"
#include "frcsim/math/quaternion.hpp"
#include "frcsim/math/vector.hpp"
#include "frcsim/rigidbody/body_flags.hpp"
#include "frcsim/rigidbody/material.hpp"

namespace frcsim {

class RigidBody {
  public:
RigidBody() = default;

explicit RigidBody(units::kilogram_t mass) {
setMass(mass);
}

void setMass(units::kilogram_t mass) {
mass_ = units::math::max(0_kg, mass);
is_static_ = (mass_ <= 0_kg);
inv_mass_ = is_static_
? units::inverse<units::kilogram_t>{0.0}
: 1.0 / mass_;
}

units::kilogram_t massKg() const { return mass_; }
units::inverse<units::kilogram_t> invMass() const { return inv_mass_; }

void setStatic(bool is_static) {
is_static_ = is_static;
inv_mass_ = is_static_
? units::inverse<units::kilogram_t>{0.0}
: ((mass_ > 0_kg) ? 1.0 / mass_ : units::inverse<units::kilogram_t>{0.0});
}

bool isStatic() const { return is_static_; }

void setBodyInertiaTensor(const Matrix3& inertia_body) {
inertia_body_ = inertia_body;
inertia_body_inv_ = inertia_body_.inverse();
}

const Matrix3& bodyInertiaTensor() const { return inertia_body_; }
const Matrix3& bodyInertiaTensorInv() const { return inertia_body_inv_; }

Matrix3 worldInertiaTensorInv() const {
const Matrix3 r = Matrix3::fromQuaternion(orientation_);
return r * inertia_body_inv_ * r.transpose();
}

const Vector3& position() const { return position_m_; }
const Quaternion& orientation() const { return orientation_; }
const Vector3& linearVelocity() const { return linear_velocity_mps_; }
const Vector3& angularVelocity() const { return angular_velocity_radps_; }

void setPosition(const Vector3& position_m) { position_m_ = position_m; }

void setOrientation(const Quaternion& orientation) {
orientation_ = orientation;
orientation_.normalizeIfNeeded();
}

void setLinearVelocity(const Vector3& linear_velocity_mps) {
linear_velocity_mps_ = linear_velocity_mps;
}

void setAngularVelocity(const Vector3& angular_velocity_radps) {
angular_velocity_radps_ = angular_velocity_radps;
}

const Vector3& accumulatedForce() const { return force_accumulator_n_; }
const Vector3& accumulatedTorque() const { return torque_accumulator_nm_; }

void clearAccumulators() {
force_accumulator_n_ = Vector3::zero();
torque_accumulator_nm_ = Vector3::zero();
}

void applyForce(const Vector3& force_n) {
if (is_static_ || flags_.is_kinematic) return;
force_accumulator_n_ += force_n;
}

void applyTorque(const Vector3& torque_nm) {
if (is_static_ || flags_.is_kinematic) return;
torque_accumulator_nm_ += torque_nm;
}

void applyForceAtPoint(const Vector3& force_n,
const Vector3& world_point_m) {
if (is_static_ || flags_.is_kinematic) return;
applyForce(force_n);
const Vector3 r = world_point_m - position_m_;
applyTorque(r.cross(force_n));
}

BodyFlags& flags() { return flags_; }
const BodyFlags& flags() const { return flags_; }

void setMaterial(const std::shared_ptr<Material>& material) {
material_ = material;
}

const std::shared_ptr<Material>& material() const { return material_; }

void setMaterial(const Material& material) {
if (!material_) {
material_ = std::make_shared<Material>(material);
} else {
*material_ = material;
}
}

void integrate(
units::second_t dt,
IntegrationMethod method,
const Vector3& gravity_mps2,
double linear_damping_per_s,
double angular_damping_per_s) {

if (is_static_ || flags_.is_kinematic) return;

const Vector3 effective_gravity =
(flags_.enable_gravity) ? gravity_mps2 : Vector3::zero();

const Vector3 linear_accel_mps2 =
force_accumulator_n_ * inv_mass_.value()
+ effective_gravity;

const Vector3 angular_accel_radps2 =
worldInertiaTensorInv() * torque_accumulator_nm_;

const double dt_s = dt.value();

switch (method) {

case IntegrationMethod::kExplicitEuler:
Integrator::integrateLinearExplicit(
position_m_,
linear_velocity_mps_,
linear_accel_mps2,
dt_s);
break;

case IntegrationMethod::kRK2:
Integrator::integrateLinearRK2(
position_m_,
linear_velocity_mps_,
linear_accel_mps2,
dt_s);
break;

case IntegrationMethod::kSemiImplicitEuler:
default:
Integrator::integrateLinear(
position_m_,
linear_velocity_mps_,
linear_accel_mps2,
dt_s);
break;
}

Integrator::integrateAngularVelocity(
angular_velocity_radps_,
angular_accel_radps2,
dt_s);

Integrator::integrateAngular(
orientation_,
angular_velocity_radps_,
dt_s);

const double linear_decay =
std::max(0.0,
1.0 - linear_damping_per_s * dt_s);

const double angular_decay =
std::max(0.0,
1.0 - angular_damping_per_s * dt_s);

// compute dimensionless decay factors using WPILib units
const units::dimensionless_t linear_decay_unit = 1.0 - linear_damping_per_s * dt;
const units::dimensionless_t angular_decay_unit = 1.0 - angular_damping_per_s * dt;

// apply the decay to velocities
linear_velocity_mps_ *= linear_decay_unit.value();
angular_velocity_radps_ *= angular_decay_unit.value();

clearAccumulators();
}

  private:

units::kilogram_t mass_{1.0};
units::inverse<units::kilogram_t> inv_mass_{1.0};

bool is_static_{false};

Matrix3 inertia_body_{};
Matrix3 inertia_body_inv_{};

Vector3 position_m_{};
Quaternion orientation_{};

Vector3 linear_velocity_mps_{};
Vector3 angular_velocity_radps_{};

Vector3 force_accumulator_n_{};
Vector3 torque_accumulator_nm_{};

BodyFlags flags_{};

std::shared_ptr<Material> material_{std::make_shared<Material>()};
};

}  // namespace frcsim