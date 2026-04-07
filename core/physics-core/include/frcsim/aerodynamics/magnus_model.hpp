#pragma once

#include "frcsim/math/vector.hpp"
#include "frcsim/rigidbody/rigid_body.hpp"

namespace frcsim {

class MagnusModel {
	public:
		explicit MagnusModel(double magnus_coefficient = 1e-4) : magnus_coefficient_(magnus_coefficient) {}

		Vector3 computeForce(const Vector3& velocity_mps, const Vector3& spin_radps) const {
				return Vector3::magnusForce(velocity_mps, spin_radps, magnus_coefficient_);
		}

		void apply(RigidBody& body) const {
				if (body.isStatic()) return;
				body.applyForce(computeForce(body.linearVelocity(), body.angularVelocity()));
		}

	private:
		double magnus_coefficient_{1e-4};
};

}  // namespace frcsim
