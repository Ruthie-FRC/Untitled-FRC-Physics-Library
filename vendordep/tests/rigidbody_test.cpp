#include <cassert>
#include <cmath>

#include "frcsim/physics_world.hpp"

int main() {
    frcsim::PhysicsConfig config;
    config.fixed_dt_s = 0.01;
    config.integration_method = frcsim::IntegrationMethod::kSemiImplicitEuler;
    config.enable_collision_detection = false;
    config.enable_joint_constraints = false;
    config.enable_aerodynamics = false;
    config.linear_damping_per_s = 0.0;
    config.angular_damping_per_s = 0.0;

    frcsim::PhysicsWorld world(config);
    frcsim::RigidBody& body = world.createBody(1.0);

    const int steps = 100;
    for (int i = 0; i < steps; ++i) {
        world.step();
    }

    // z should decrease under gravity.
    assert(body.position().z < 0.0);

    // Compare velocity to v = g*t with damping disabled for test accuracy.
    // TODO: If damping defaults change, set them explicitly in this test config.
    const double t = steps * config.fixed_dt_s;
    const double expected_vz = config.gravity_mps2.z * t;
    const double tolerance = 0.1;
    assert(std::fabs(body.linearVelocity().z - expected_vz) < tolerance);

    return 0;
}
