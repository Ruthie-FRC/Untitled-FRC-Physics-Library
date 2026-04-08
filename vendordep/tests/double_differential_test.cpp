#include <cassert>
#include <cmath>

#include "frcsim/mechanisms/double_differential.hpp"

int main() {
    frcsim::DoubleDifferentialMechanism::Config config;
    config.m00 = 0.5;
    config.m01 = 0.5;
    config.m10 = 0.5;
    config.m11 = -0.5;
    config.yaw_scale = 1.0;
    config.pitch_scale = 1.0;
    config.limits.min_yaw_rad = -2.0;
    config.limits.max_yaw_rad = 2.0;
    config.limits.min_pitch_rad = -1.2;
    config.limits.max_pitch_rad = 1.2;

    frcsim::DoubleDifferentialMechanism mechanism(config);

    // Forward kinematics for an ideal differential mapping.
    frcsim::DoubleDifferentialMechanism::MotorState motor_state;
    motor_state.motor_a_position_rad = 1.0;
    motor_state.motor_b_position_rad = 0.2;
    motor_state.motor_a_velocity_radps = 0.6;
    motor_state.motor_b_velocity_radps = -0.2;

    const auto joints = mechanism.forward(motor_state);
    assert(std::fabs(joints.yaw_rad - 0.6) < 1e-9);
    assert(std::fabs(joints.pitch_rad - 0.4) < 1e-9);
    assert(std::fabs(joints.yaw_velocity_radps - 0.2) < 1e-9);
    assert(std::fabs(joints.pitch_velocity_radps - 0.4) < 1e-9);

    // Inverse should recover original motor state for a non-singular mapping.
    const auto inverse_result = mechanism.inverse(joints);
    assert(inverse_result.valid);
    assert(std::fabs(inverse_result.motor_state.motor_a_position_rad - motor_state.motor_a_position_rad) < 1e-9);
    assert(std::fabs(inverse_result.motor_state.motor_b_position_rad - motor_state.motor_b_position_rad) < 1e-9);
    assert(std::fabs(inverse_result.motor_state.motor_a_velocity_radps - motor_state.motor_a_velocity_radps) < 1e-9);
    assert(std::fabs(inverse_result.motor_state.motor_b_velocity_radps - motor_state.motor_b_velocity_radps) < 1e-9);

    // Desired state should clamp to configured joint limits before inverse mapping.
    frcsim::DoubleDifferentialMechanism::JointState out_of_bounds;
    out_of_bounds.yaw_rad = 3.0;
    out_of_bounds.pitch_rad = -2.0;

    const auto clamped_inverse = mechanism.inverse(out_of_bounds);
    assert(clamped_inverse.valid);
    const auto clamped_joints = mechanism.forward(clamped_inverse.motor_state);
    assert(clamped_joints.yaw_rad <= config.limits.max_yaw_rad + 1e-9);
    assert(clamped_joints.pitch_rad >= config.limits.min_pitch_rad - 1e-9);

    // Singular matrix must be reported as invalid.
    frcsim::DoubleDifferentialMechanism::Config singular_config;
    singular_config.m00 = 1.0;
    singular_config.m01 = 1.0;
    singular_config.m10 = 2.0;
    singular_config.m11 = 2.0;  // second row is linearly dependent.

    frcsim::DoubleDifferentialMechanism singular_mechanism(singular_config);
    const auto singular_result = singular_mechanism.inverse(joints);
    assert(!singular_result.valid);

    return 0;
}
