#pragma once

#include <algorithm>

#include "frcsim/gamepiece/ball_physics.hpp"
#include "frcsim/math/quaternion.hpp"
#include "frcsim/math/vector.hpp"
#include "frcsim/mechanisms/double_differential.hpp"

namespace frcsim {

class TurretShooterSim {
  public:
    struct Config {
        DoubleDifferentialMechanism::Config differential{};
        BallPhysicsSim3D::Config ball_config{};
        BallPhysicsSim3D::BallProperties ball_properties{};

        // Position of the differential pivot in world frame.
        Vector3 turret_mount_position_m{};

        // Muzzle origin in local turret frame before yaw/pitch rotation.
        Vector3 muzzle_offset_local_m{0.35, 0.0, 0.18};

        // Intake center in local turret frame.
        Vector3 intake_offset_local_m{0.15, 0.0, -0.06};

        double intake_capture_radius_m{0.22};
        Vector3 carry_offset_local_m{0.05, 0.0, -0.02};
    };

    TurretShooterSim() = default;

    explicit TurretShooterSim(const Config& config)
        : config_(config), differential_(config.differential), ball_(config.ball_config, config.ball_properties) {}

    const Config& config() const { return config_; }

    void setTurretMountPosition(const Vector3& mount_position_m) {
        config_.turret_mount_position_m = mount_position_m;
    }

    void setBaseVelocity(const Vector3& base_velocity_mps) {
        base_velocity_mps_ = base_velocity_mps;
    }

    void setMotorState(const DoubleDifferentialMechanism::MotorState& motor_state) {
        motor_state_ = motor_state;
    }

    const DoubleDifferentialMechanism::MotorState& motorState() const {
        return motor_state_;
    }

    DoubleDifferentialMechanism::InverseResult solveMotorStateForAim(double yaw_rad, double pitch_rad,
                                                                      double yaw_velocity_radps = 0.0,
                                                                      double pitch_velocity_radps = 0.0) const {
        DoubleDifferentialMechanism::JointState desired{};
        desired.yaw_rad = yaw_rad;
        desired.pitch_rad = pitch_rad;
        desired.yaw_velocity_radps = yaw_velocity_radps;
        desired.pitch_velocity_radps = pitch_velocity_radps;
        return differential_.inverse(desired);
    }

    bool applyAim(double yaw_rad, double pitch_rad, double yaw_velocity_radps = 0.0,
                  double pitch_velocity_radps = 0.0) {
        const auto solved = solveMotorStateForAim(yaw_rad, pitch_rad, yaw_velocity_radps, pitch_velocity_radps);
        if (!solved.valid) {
            return false;
        }
        motor_state_ = solved.motor_state;
        return true;
    }

    DoubleDifferentialMechanism::JointState jointState() const {
        return differential_.forward(motor_state_);
    }

    Vector3 muzzlePositionWorld() const {
        return config_.turret_mount_position_m + turretOrientationWorld().rotate(config_.muzzle_offset_local_m);
    }

    Vector3 intakePositionWorld() const {
        return config_.turret_mount_position_m + turretOrientationWorld().rotate(config_.intake_offset_local_m);
    }

    Vector3 muzzleDirectionWorld() const {
        return turretOrientationWorld().rotate(Vector3::unitX()).normalized();
    }

    bool pickupBallNearby() {
        BallPhysicsSim3D::PickupRequest request{};
        request.intake_position_m = intakePositionWorld();
        request.capture_radius_m = config_.intake_capture_radius_m;
        request.carry_offset_m = turretOrientationWorld().rotate(config_.carry_offset_local_m);
        return ball_.requestPickup(request);
    }

    void shoot(double muzzle_speed_mps, const Vector3& spin_radps = Vector3::zero()) {
        const Vector3 muzzle_position = muzzlePositionWorld();
        const Vector3 shot_velocity = base_velocity_mps_ + muzzleDirectionWorld() * std::max(0.0, muzzle_speed_mps);
        ball_.shoot(muzzle_position, shot_velocity, spin_radps);
    }

    void step(double dt_s) {
        const Vector3 carry_position = intakePositionWorld();
        ball_.setCarrierPose(carry_position, base_velocity_mps_);
        ball_.step(dt_s);
    }

    BallPhysicsSim3D& ball() { return ball_; }
    const BallPhysicsSim3D& ball() const { return ball_; }

  private:
    Quaternion turretOrientationWorld() const {
        const auto joints = jointState();
        const Quaternion yaw = Quaternion::fromAxisAngle(Vector3::unitZ(), joints.yaw_rad);
        const Quaternion pitch = Quaternion::fromAxisAngle(Vector3::unitY(), joints.pitch_rad);
        return yaw * pitch;
    }

    Config config_{};
    DoubleDifferentialMechanism differential_{};
    DoubleDifferentialMechanism::MotorState motor_state_{};

    Vector3 base_velocity_mps_{};
    BallPhysicsSim3D ball_{};
};

}  // namespace frcsim
