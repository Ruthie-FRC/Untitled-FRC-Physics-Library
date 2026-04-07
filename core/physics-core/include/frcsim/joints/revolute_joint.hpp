#pragma once

#include "frcsim/joints/joint_base.hpp"
#include "frcsim/math/vector.hpp"

namespace frcsim {

class RigidBody;

class RevoluteJoint : public JointBase {
  public:
    RevoluteJoint(RigidBody* body_a, RigidBody* body_b, const Vector3& axis_local)
        : JointBase(JointType::kRevolute, body_a, body_b), axis_local_(axis_local.normalized()) {}

    const Vector3& axisLocal() const { return axis_local_; }
    void setAxisLocal(const Vector3& axis) { axis_local_ = axis.normalized(); }

    void setLimits(double min_angle_rad, double max_angle_rad) {
        min_angle_rad_ = min_angle_rad;
        max_angle_rad_ = max_angle_rad;
        has_limits_ = true;
    }

    void clearLimits() { has_limits_ = false; }
    bool hasLimits() const { return has_limits_; }
    double minAngle() const { return min_angle_rad_; }
    double maxAngle() const { return max_angle_rad_; }

    void setMotorTarget(double target_velocity_radps, double max_torque_nm) {
        motor_target_velocity_radps_ = target_velocity_radps;
        motor_max_torque_nm_ = std::max(0.0, max_torque_nm);
        has_motor_ = (motor_max_torque_nm_ > 0.0);
    }

    bool hasMotor() const { return has_motor_; }
    double motorTargetVelocity() const { return motor_target_velocity_radps_; }
    double motorMaxTorque() const { return motor_max_torque_nm_; }

    void solveConstraint(double dt_s, int iterations) override;
    double constraintError() const override;

  private:
    Vector3 axis_local_;
    bool has_limits_{false};
    double min_angle_rad_{0.0};
    double max_angle_rad_{0.0};

    bool has_motor_{false};
    double motor_target_velocity_radps_{0.0};
    double motor_max_torque_nm_{0.0};

    double cached_impulse_{0.0};
};

}  // namespace frcsim
