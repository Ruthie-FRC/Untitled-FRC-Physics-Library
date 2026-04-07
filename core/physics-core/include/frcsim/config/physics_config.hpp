#pragma once

#include "frcsim/math/vector.hpp"

namespace frcsim {

enum class IntegrationMethod {
	kExplicitEuler,
	kSemiImplicitEuler,
	kRK2,
};

struct PhysicsConfig {
	double fixed_dt_s{0.005};
	bool deterministic_stepping{true};
	IntegrationMethod integration_method{IntegrationMethod::kSemiImplicitEuler};

	bool enable_collision_detection{true};
	bool enable_joint_constraints{true};
	bool enable_gravity{true};
	bool enable_aerodynamics{true};

	Vector3 gravity_mps2{0.0, 0.0, -9.80665};

	double linear_damping_per_s{0.05};
	double angular_damping_per_s{0.10};

	// Iteration count for iterative solvers (contacts/joints).
	// TODO: Tune for target real-time budget and mechanism stability goals.
	int solver_iterations{8};
};

}  // namespace frcsim
