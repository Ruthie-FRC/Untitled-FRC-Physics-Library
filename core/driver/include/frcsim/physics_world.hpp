#pragma once

#include <cstddef>
#include <memory>
#include <vector>

#include "frcsim/field/boundary.hpp"
#include "frcsim/forces/force_generator.hpp"
#include "frcsim/rigidbody/rigid_assembly.hpp"
#include "frcsim/rigidbody/rigid_body.hpp"

namespace frcsim {

struct PhysicsConfig {
		double fixed_dt_s{0.01};
		IntegrationMethod integration_method{IntegrationMethod::kSemiImplicitEuler};

		bool enable_collision_detection{false};
		bool enable_joint_constraints{false};
		bool enable_aerodynamics{false};
		bool enable_gravity{true};

		Vector3 gravity_mps2{0.0, 0.0, -9.81};
		double linear_damping_per_s{0.0};
		double angular_damping_per_s{0.0};
};

class PhysicsWorld {
	public:
		explicit PhysicsWorld(const PhysicsConfig& config = PhysicsConfig()) : config_(config) {}

		RigidBody& createBody(double mass_kg);
		RigidAssembly& createAssembly();

		EnvironmentalBoundary& addBoundary();
		std::vector<EnvironmentalBoundary>& boundaries() { return boundaries_; }
		const std::vector<EnvironmentalBoundary>& boundaries() const { return boundaries_; }

		void addGlobalForceGenerator(const std::shared_ptr<ForceGenerator>& generator);

		void step();

		std::size_t stepCount() const { return step_count_; }
		double accumulatedSimTimeS() const { return accumulated_sim_time_s_; }

		PhysicsConfig& config() { return config_; }
		const PhysicsConfig& config() const { return config_; }

	private:
		PhysicsConfig config_{};

		std::vector<RigidBody> bodies_{};
		std::vector<RigidAssembly> assemblies_{};
		std::vector<EnvironmentalBoundary> boundaries_{};
		std::vector<std::shared_ptr<ForceGenerator>> global_force_generators_{};

		std::size_t step_count_{0};
		double accumulated_sim_time_s_{0.0};
};

}  // namespace frcsim
