#pragma once

#include <optional>

#include "frcsim/math/integrators.hpp"
#include "frcsim/math/matrix.hpp"
#include "frcsim/math/quaternion.hpp"
#include "frcsim/math/vector.hpp"
#include "frcsim/rigidbody/body_flags.hpp"
#include "frcsim/rigidbody/material.hpp"

namespace frcsim {

enum class IntegrationMethod {
	kSemiImplicitEuler,
	kExplicitEuler,
	kRK2,
};

class RigidBody {
  public:
	explicit RigidBody(double mass_kg = 1.0) { setMassKg(mass_kg); }

	double massKg() const { return mass_kg_; }
	double inverseMass() const { return inv_mass_; }
	void setMassKg(double mass_kg) {
		mass_kg_ = (mass_kg > 0.0) ? mass_kg : 1.0;
		inv_mass_ = 1.0 / mass_kg_;
	}

	const Vector3& position() const { return position_m_; }
	void setPosition(const Vector3& position_m) { position_m_ = position_m; }

	const Quaternion& orientation() const { return orientation_; }
	void setOrientation(const Quaternion& orientation) { orientation_ = orientation; }

	const Vector3& linearVelocity() const { return linear_velocity_mps_; }
	void setLinearVelocity(const Vector3& velocity_mps) { linear_velocity_mps_ = velocity_mps; }

	const Vector3& angularVelocity() const { return angular_velocity_radps_; }
	void setAngularVelocity(const Vector3& angular_velocity_radps) {
		angular_velocity_radps_ = angular_velocity_radps;
	}

	const Matrix3& bodyInertiaTensor() const { return body_inertia_tensor_; }
	void setBodyInertiaTensor(const Matrix3& inertia) {
		body_inertia_tensor_ = inertia;
		inv_body_inertia_tensor_ = inertia.inverse();
	}

	BodyFlags& flags() { return flags_; }
	const BodyFlags& flags() const { return flags_; }

	void setMaterial(const Material& material) { material_ = material; }
	const Material* material() const { return material_ ? &(*material_) : nullptr; }

	void applyForce(const Vector3& force_n) { accumulated_force_n_ += force_n; }

	void applyForceAtPoint(const Vector3& force_n, const Vector3& world_point_m) {
		applyForce(force_n);
		const Vector3 r = world_point_m - position_m_;
		accumulated_torque_nm_ += r.cross(force_n);
	}

	void clearAccumulators() {
		accumulated_force_n_ = Vector3::zero();
		accumulated_torque_nm_ = Vector3::zero();
	}

	void integrate(double dt_s, IntegrationMethod method, const Vector3& gravity_mps2,
				   double linear_damping_per_s, double angular_damping_per_s) {
		if (flags_.is_kinematic) {
			clearAccumulators();
			return;
		}

		Vector3 total_force = accumulated_force_n_;
		if (flags_.enable_gravity) {
			total_force += gravity_mps2 * mass_kg_;
		}

		const Vector3 linear_accel = total_force * inv_mass_;
		switch (method) {
			case IntegrationMethod::kExplicitEuler:
				Integrator::integrateLinearExplicit(position_m_, linear_velocity_mps_, linear_accel, dt_s);
				break;
			case IntegrationMethod::kRK2:
				Integrator::integrateLinearRK2(position_m_, linear_velocity_mps_, linear_accel, dt_s);
				break;
			case IntegrationMethod::kSemiImplicitEuler:
			default:
				Integrator::integrateLinear(position_m_, linear_velocity_mps_, linear_accel, dt_s);
				break;
		}

		const double linear_damp = std::max(0.0, 1.0 - linear_damping_per_s * dt_s);
		linear_velocity_mps_ *= linear_damp;

		const Vector3 angular_accel = inv_body_inertia_tensor_ * accumulated_torque_nm_;
		Integrator::integrateAngularVelocity(angular_velocity_radps_, angular_accel, dt_s);
		Integrator::integrateAngular(orientation_, angular_velocity_radps_, dt_s);

		const double angular_damp = std::max(0.0, 1.0 - angular_damping_per_s * dt_s);
		angular_velocity_radps_ *= angular_damp;

		clearAccumulators();
	}

  private:
	double mass_kg_{1.0};
	double inv_mass_{1.0};

	Vector3 position_m_{};
	Quaternion orientation_{};
	Vector3 linear_velocity_mps_{};
	Vector3 angular_velocity_radps_{};

	Vector3 accumulated_force_n_{};
	Vector3 accumulated_torque_nm_{};

	Matrix3 body_inertia_tensor_{Matrix3::identity()};
	Matrix3 inv_body_inertia_tensor_{Matrix3::identity()};

	BodyFlags flags_{};
	std::optional<Material> material_{};
};

}  // namespace frcsim
