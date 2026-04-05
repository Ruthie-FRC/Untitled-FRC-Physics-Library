#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

namespace frcsim {

class DoubleDifferentialMechanism {
  public:
    struct Limits {
        double min_yaw_rad{-std::numeric_limits<double>::infinity()};
        double max_yaw_rad{std::numeric_limits<double>::infinity()};
        double min_pitch_rad{-std::numeric_limits<double>::infinity()};
        double max_pitch_rad{std::numeric_limits<double>::infinity()};
    };

    struct Config {
        // Mapping from motor shaft position [a, b] to joint coordinates [yaw, pitch].
        // yaw = m00 * a + m01 * b
        // pitch = m10 * a + m11 * b
        double m00{0.5};
        double m01{0.5};
        double m10{0.5};
        double m11{-0.5};

        // Optional multiplicative scale factors after matrix mapping.
        double yaw_scale{1.0};
        double pitch_scale{1.0};

        Limits limits{};
        double singularity_epsilon{1e-9};
    };

    struct MotorState {
        double motor_a_position_rad{0.0};
        double motor_b_position_rad{0.0};
        double motor_a_velocity_radps{0.0};
        double motor_b_velocity_radps{0.0};
    };

    struct JointState {
        double yaw_rad{0.0};
        double pitch_rad{0.0};
        double yaw_velocity_radps{0.0};
        double pitch_velocity_radps{0.0};
    };

    struct InverseResult {
        MotorState motor_state{};
        bool valid{true};
    };

    DoubleDifferentialMechanism() = default;

    explicit DoubleDifferentialMechanism(const Config& config) : config_(config) {}

    const Config& config() const { return config_; }
    void setConfig(const Config& config) { config_ = config; }

    JointState forward(const MotorState& motor_state) const {
        JointState out{};
        out.yaw_rad = config_.yaw_scale * (config_.m00 * motor_state.motor_a_position_rad +
                                           config_.m01 * motor_state.motor_b_position_rad);
        out.pitch_rad = config_.pitch_scale * (config_.m10 * motor_state.motor_a_position_rad +
                                               config_.m11 * motor_state.motor_b_position_rad);

        out.yaw_velocity_radps =
            config_.yaw_scale * (config_.m00 * motor_state.motor_a_velocity_radps +
                                 config_.m01 * motor_state.motor_b_velocity_radps);
        out.pitch_velocity_radps =
            config_.pitch_scale * (config_.m10 * motor_state.motor_a_velocity_radps +
                                   config_.m11 * motor_state.motor_b_velocity_radps);

        applyLimits(out);
        return out;
    }

    InverseResult inverse(const JointState& desired_joint_state) const {
        InverseResult result{};

        const double a11 = config_.yaw_scale * config_.m00;
        const double a12 = config_.yaw_scale * config_.m01;
        const double a21 = config_.pitch_scale * config_.m10;
        const double a22 = config_.pitch_scale * config_.m11;

        const double det = a11 * a22 - a12 * a21;
        if (std::abs(det) <= config_.singularity_epsilon) {
            result.valid = false;
            return result;
        }

        const double inv11 = a22 / det;
        const double inv12 = -a12 / det;
        const double inv21 = -a21 / det;
        const double inv22 = a11 / det;

        const JointState clamped = clampedToLimits(desired_joint_state);

        result.motor_state.motor_a_position_rad = inv11 * clamped.yaw_rad + inv12 * clamped.pitch_rad;
        result.motor_state.motor_b_position_rad = inv21 * clamped.yaw_rad + inv22 * clamped.pitch_rad;

        result.motor_state.motor_a_velocity_radps =
            inv11 * clamped.yaw_velocity_radps + inv12 * clamped.pitch_velocity_radps;
        result.motor_state.motor_b_velocity_radps =
            inv21 * clamped.yaw_velocity_radps + inv22 * clamped.pitch_velocity_radps;

        return result;
    }

    JointState clampedToLimits(const JointState& state) const {
        JointState out = state;
        applyLimits(out);
        return out;
    }

  private:
    void applyLimits(JointState& state) const {
        state.yaw_rad = std::clamp(state.yaw_rad, config_.limits.min_yaw_rad, config_.limits.max_yaw_rad);
        state.pitch_rad =
            std::clamp(state.pitch_rad, config_.limits.min_pitch_rad, config_.limits.max_pitch_rad);
    }

    Config config_{};
};

}  // namespace frcsim
