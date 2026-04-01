#pragma once

#include "frcsim/forces/force_generator.hpp"
#include "frcsim/math/vector.hpp"
#include "frcsim/rigidbody/rigid_body.hpp"

namespace frcsim {

class GravityForce : public ForceGenerator {
	public:
		explicit GravityForce(const Vector3& gravity_mps2) : gravity_mps2_(gravity_mps2) {}

		void apply(RigidBody& body, double /*dt_s*/) const override {
				if (body.flags().is_kinematic) return;
				body.applyForce(gravity_mps2_ * body.massKg());
		}

	private:
		Vector3 gravity_mps2_{};
};

}  // namespace frcsim
