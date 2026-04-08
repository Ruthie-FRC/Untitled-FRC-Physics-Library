// Copyright (c) RenSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

#include <algorithm>

#include "frcsim/gamepiece/ball_physics.hpp"
#include "frcsim/math/quaternion.hpp"
#include "frcsim/math/vector.hpp"
#include "frcsim/mechanisms/double_differential.hpp"
#include "frcsim/mechanisms/shooter_wheel.hpp"

namespace frcsim {

/** @brief Integrated turret + shooter + carried-ball simulator for aiming,
 * pickup, and firing flows. */
class TurretShooterSim {
 public:
  /** @brief Configuration for differential kinematics, ball model, and local
   * mount geometry. */
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

  /**
   * @brief Constructs simulator from explicit configuration.
   * @param config Initial turret/shooter/ball configuration.
   */
  explicit TurretShooterSim(const Config& config)
      : config_(config),
        differential_(config.differential),
        ball_(config.ball_config, config.ball_properties) {}

  /** @brief Returns current simulator configuration. @return Immutable config
   * reference. */
  const Config& config() const { return config_; }

  /** @brief Updates turret mount origin in world frame. @param mount_position_m
   * Mount position in meters. */
  void setTurretMountPosition(const Vector3& mount_position_m) {
    config_.turret_mount_position_m = mount_position_m;
  }

  /** @brief Sets base/chassis linear velocity used during carry and shot
   * composition. @param base_velocity_mps World velocity. */
  void setBaseVelocity(const Vector3& base_velocity_mps) {
    base_velocity_mps_ = base_velocity_mps;
  }

  /** @brief Sets differential motor state directly. @param motor_state Motor
   * positions/velocities. */
  void setMotorState(
      const DoubleDifferentialMechanism::MotorState& motor_state) {
    motor_state_ = motor_state;
  }

  /** @brief Returns current differential motor state. @return Immutable motor
   * state reference. */
  const DoubleDifferentialMechanism::MotorState& motorState() const {
    return motor_state_;
  }

  /**
   * @brief Solves inverse differential mapping for desired aim state.
   * @param yaw_rad Desired yaw angle in radians.
   * @param pitch_rad Desired pitch angle in radians.
   * @param yaw_velocity_radps Desired yaw rate in rad/s.
   * @param pitch_velocity_radps Desired pitch rate in rad/s.
   * @return InverseResult containing motor command and validity flag.
   */
  DoubleDifferentialMechanism::InverseResult solveMotorStateForAim(
      double yaw_rad, double pitch_rad, double yaw_velocity_radps = 0.0,
      double pitch_velocity_radps = 0.0) const {
    DoubleDifferentialMechanism::JointState desired{};
    desired.yaw_rad = yaw_rad;
    desired.pitch_rad = pitch_rad;
    desired.yaw_velocity_radps = yaw_velocity_radps;
    desired.pitch_velocity_radps = pitch_velocity_radps;
    return differential_.inverse(desired);
  }

  /**
   * @brief Applies desired aim by solving and storing motor state.
   * @param yaw_rad Desired yaw angle in radians.
   * @param pitch_rad Desired pitch angle in radians.
   * @param yaw_velocity_radps Desired yaw rate in rad/s.
   * @param pitch_velocity_radps Desired pitch rate in rad/s.
   * @return True when inverse solve succeeded.
   */
  bool applyAim(double yaw_rad, double pitch_rad,
                double yaw_velocity_radps = 0.0,
                double pitch_velocity_radps = 0.0) {
    const auto solved = solveMotorStateForAim(
        yaw_rad, pitch_rad, yaw_velocity_radps, pitch_velocity_radps);
    if (!solved.valid) {
      return false;
    }
    motor_state_ = solved.motor_state;
    return true;
  }

  /** @brief Computes current turret joint state from motor state. @return
   * JointState with yaw/pitch and rates. */
  DoubleDifferentialMechanism::JointState jointState() const {
    return differential_.forward(motor_state_);
  }

  /** @brief Returns current muzzle origin in world coordinates. @return World
   * position of muzzle. */
  Vector3 muzzlePositionWorld() const {
    return config_.turret_mount_position_m +
           turretOrientationWorld().rotate(config_.muzzle_offset_local_m);
  }

  /** @brief Returns intake center in world coordinates. @return World position
   * of intake center. */
  Vector3 intakePositionWorld() const {
    return config_.turret_mount_position_m +
           turretOrientationWorld().rotate(config_.intake_offset_local_m);
  }

  /** @brief Returns normalized muzzle forward direction in world frame. @return
   * Unit direction vector. */
  Vector3 muzzleDirectionWorld() const {
    return turretOrientationWorld().rotate(Vector3::unitX()).normalized();
  }

  /** @brief Attempts to pick up the internal ball if near intake location.
   * @return True if pickup succeeds. */
  bool pickupBallNearby() {
    BallPhysicsSim3D::PickupRequest request{};
    request.intake_position_m = intakePositionWorld();
    request.capture_radius_m = config_.intake_capture_radius_m;
    request.carry_offset_m =
        turretOrientationWorld().rotate(config_.carry_offset_local_m);
    return ball_.requestPickup(request);
  }

  /**
   * @brief Fires the ball using an explicit muzzle speed.
   * @param muzzle_speed_mps Launch speed in m/s.
   * @param spin_radps Launch spin vector in rad/s.
   */
  void shoot(double muzzle_speed_mps,
             const Vector3& spin_radps = Vector3::zero()) {
    const Vector3 muzzle_position = muzzlePositionWorld();
    const Vector3 shot_velocity =
        base_velocity_mps_ +
        muzzleDirectionWorld() * std::max(0.0, muzzle_speed_mps);
    ball_.shoot(muzzle_position, shot_velocity, spin_radps);
  }

  /**
   * @brief Fires the ball using wheel-estimated exit velocity.
   * @param wheel Shooter wheel model used for speed estimate.
   * @param spin_radps Launch spin vector in rad/s.
   */
  void shootWithWheel(const ShooterWheelSim& wheel,
                      const Vector3& spin_radps = Vector3::zero()) {
    shoot(wheel.estimatedExitVelocityMps(), spin_radps);
  }

  /**
   * @brief Advances carry/ball simulation one step.
   * @param dt_s Timestep in seconds.
   */
  void step(double dt_s) {
    const Vector3 carry_position = intakePositionWorld();
    ball_.setCarrierPose(carry_position, base_velocity_mps_);
    ball_.step(dt_s);
  }

  /** @brief Mutable access to internal ball simulator. @return BallPhysicsSim3D
   * reference. */
  BallPhysicsSim3D& ball() { return ball_; }
  /** @brief Immutable access to internal ball simulator. @return Const
   * BallPhysicsSim3D reference. */
  const BallPhysicsSim3D& ball() const { return ball_; }

 private:
  Quaternion turretOrientationWorld() const {
    const auto joints = jointState();
    const Quaternion yaw =
        Quaternion::fromAxisAngle(Vector3::unitZ(), joints.yaw_rad);
    const Quaternion pitch =
        Quaternion::fromAxisAngle(Vector3::unitY(), joints.pitch_rad);
    return yaw * pitch;
  }

  Config config_{};
  DoubleDifferentialMechanism differential_{};
  DoubleDifferentialMechanism::MotorState motor_state_{};

  Vector3 base_velocity_mps_{};
  BallPhysicsSim3D ball_{};
};

}  // namespace frcsim
