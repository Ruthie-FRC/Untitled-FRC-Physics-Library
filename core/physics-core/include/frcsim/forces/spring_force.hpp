#pragma once

#include "frcsim/forces/force_generator.hpp"
#include "frcsim/math/vector.hpp"
#include "frcsim/rigidbody/rigid_body.hpp"

namespace frcsim {

class SpringForce final : public ForceGenerator {
	public:
		SpringForce(
				const Vector3& anchor_world_m,
				const Vector3& attach_local_m,
				double rest_length_m,
				double stiffness_npm,
				double damping_nspm)
				: anchor_world_m_(anchor_world_m),
					attach_local_m_(attach_local_m),
					rest_length_m_(rest_length_m),
					stiffness_npm_(stiffness_npm),
					damping_nspm_(damping_nspm) {}

		void apply(RigidBody& body, double /*dt_s*/) const override {
				if (body.isStatic()) return;

				const Vector3 attach_world_m = body.position() + body.orientation().rotate(attach_local_m_);
				const Vector3 r = attach_world_m - body.position();
				const Vector3 delta = attach_world_m - anchor_world_m_;
				const double length_m = delta.norm();
				if (length_m <= 1e-12) return;

				const Vector3 dir = delta / length_m;
				const Vector3 point_velocity_mps = body.linearVelocity() + body.angularVelocity().cross(r);
				const double extension_m = length_m - rest_length_m_;
				const double relative_speed_mps = point_velocity_mps.dot(dir);
				const double spring_force_n = -stiffness_npm_ * extension_m - damping_nspm_ * relative_speed_mps;

				body.applyForceAtPoint(dir * spring_force_n, attach_world_m);
		}

	private:
		Vector3 anchor_world_m_;
		Vector3 attach_local_m_;
		double rest_length_m_{0.0};
		double stiffness_npm_{0.0};
		double damping_nspm_{0.0};
};

}  // namespace frcsim
