#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

#include "frcsim/field/boundary.hpp"
#include "frcsim/gamepiece/ball_physics.hpp"
#include "frcsim/math/quaternion.hpp"
#include "frcsim/math/vector.hpp"

namespace frcsim {

class BallGamepieceSim {
  public:
    static constexpr std::size_t kNoBall = static_cast<std::size_t>(-1);

    struct FieldConfig {
        Vector3 min_corner_m{0.0, 0.0, 0.0};
        Vector3 max_corner_m{16.54, 8.21, 3.0};
        double wall_restitution{0.25};
        double wall_friction{0.2};

        // If >= 0, box boundaries with this user_id behave as net/catcher volumes.
        int net_boundary_user_id{-1};
        double net_entry_slack_scale{0.7};
        double net_velocity_decay{0.2};
        double net_spin_decay{0.8};
        double net_downward_bias_mps2{2.0};

        // Robot-ball contact/plowing tuning.
        double robot_ball_contact_restitution{0.45};
        double robot_ball_contact_friction{0.2};
        double plow_ball_velocity_retention{0.7};
        double plow_robot_velocity_gain{0.6};
    };

    struct RobotState {
        Vector3 position_m{};
        Vector3 velocity_mps{};
        double yaw_rad{0.0};

        double radius_m{0.45};
        bool intake_enabled{false};
        std::size_t intake_capacity{1};
        double intake_radius_m{0.28};
        Vector3 intake_offset_m{0.45, 0.0, 0.10};
        Vector3 carry_offset_m{0.25, 0.0, 0.25};

        std::size_t carried_ball_index{kNoBall};
    };

    struct FireCommand {
        // Relative launch translation from robot center in robot frame.
        Vector3 launch_offset_m{0.45, 0.0, 0.55};

        // Launch angles in robot frame.
        double yaw_offset_rad{0.0};
        double pitch_rad{0.8};

        // Mechanism speed scalar and measured/estimated exit scalar.
        // If estimated_exit_velocity_mps > 0, it is used as launch scalar.
        double mechanism_speed_mps{14.0};
        double estimated_exit_velocity_mps{-1.0};

        Vector3 spin_radps{};
    };

    struct BallEntity {
        BallPhysicsSim3D sim{};
        bool scored_in_net{false};
    };

    BallGamepieceSim() = default;

    explicit BallGamepieceSim(const FieldConfig& field) : field_(field) {}

    static BallPhysicsSim3D::BallProperties defaultBallProperties() {
        BallPhysicsSim3D::BallProperties properties{};
        properties.mass_kg = 0.24;
        properties.radius_m = 0.09;
        properties.drag_coefficient = 0.50;
        properties.reference_area_m2 = 3.14159265358979323846 * properties.radius_m * properties.radius_m;
        properties.restitution = 0.48;
        return properties;
    }

    static BallPhysicsSim3D::Config defaultBallConfig() {
        BallPhysicsSim3D::Config config{};
        config.gravity_mps2 = Vector3(0.0, 0.0, -9.81);
        config.air_density_kgpm3 = 1.225;
        config.magnus_coefficient = 1.0e-4;
        config.ground_height_m = 0.0;
        config.rolling_friction_per_s = 1.5;
        config.min_bounce_speed_mps = 0.06;
        return config;
    }

    // 2026 preset: 15 cm diameter, 0.216 kg, high-density foam.
    static BallPhysicsSim3D::BallProperties season2026BallProperties() {
        BallPhysicsSim3D::BallProperties properties{};
        properties.mass_kg = 0.216;
        properties.radius_m = 0.15 * 0.5;
        properties.drag_coefficient = 0.58;
        properties.reference_area_m2 = 3.14159265358979323846 * properties.radius_m * properties.radius_m;
        properties.restitution = 0.52;
        return properties;
    }

    static BallPhysicsSim3D::Config season2026BallConfig() {
        BallPhysicsSim3D::Config config = defaultBallConfig();
        config.magnus_coefficient = 1.2e-4;
        config.rolling_friction_per_s = 1.8;
        return config;
    }

    std::size_t addRobot(const RobotState& robot) {
        robots_.push_back(robot);
        return robots_.size() - 1;
    }

    std::size_t addBall(const BallPhysicsSim3D::BallState& state,
                        const BallPhysicsSim3D::Config& config = defaultBallConfig(),
                        const BallPhysicsSim3D::BallProperties& properties = defaultBallProperties()) {
        BallEntity entity{};
        entity.sim = BallPhysicsSim3D(config, properties);
        entity.sim.setState(state);
        balls_.push_back(entity);
        return balls_.size() - 1;
    }

    EnvironmentalBoundary& addFieldElement(const EnvironmentalBoundary& boundary) {
        field_elements_.push_back(boundary);
        return field_elements_.back();
    }

    std::vector<RobotState>& robots() { return robots_; }
    const std::vector<RobotState>& robots() const { return robots_; }

    std::vector<BallEntity>& balls() { return balls_; }
    const std::vector<BallEntity>& balls() const { return balls_; }

    std::vector<EnvironmentalBoundary>& fieldElements() { return field_elements_; }
    const std::vector<EnvironmentalBoundary>& fieldElements() const { return field_elements_; }

    std::size_t countBalls() const { return balls_.size(); }

    std::size_t countScoredBalls() const {
        std::size_t count = 0;
        for (const auto& ball : balls_) {
            if (ball.scored_in_net) {
                ++count;
            }
        }
        return count;
    }

    bool fireBall(std::size_t robot_index, const FireCommand& command) {
        if (robot_index >= robots_.size()) {
            return false;
        }

        RobotState& robot = robots_[robot_index];
        if (robot.carried_ball_index == kNoBall || robot.carried_ball_index >= balls_.size()) {
            return false;
        }

        BallEntity& ball = balls_[robot.carried_ball_index];
        const double launch_speed = command.estimated_exit_velocity_mps > 0.0
                                        ? command.estimated_exit_velocity_mps
                                        : std::max(0.0, command.mechanism_speed_mps);

        const double yaw_world = robot.yaw_rad + command.yaw_offset_rad;
        const double cos_pitch = std::cos(command.pitch_rad);
        const Vector3 direction_world(std::cos(yaw_world) * cos_pitch,
                                      std::sin(yaw_world) * cos_pitch,
                                      std::sin(command.pitch_rad));

        const Vector3 launch_position = robot.position_m + rotateYaw(command.launch_offset_m, robot.yaw_rad);
        const Vector3 launch_velocity = robot.velocity_mps + direction_world * launch_speed;

        ball.sim.shoot(launch_position, launch_velocity, command.spin_radps);
        ball.scored_in_net = false;
        robot.carried_ball_index = kNoBall;
        return true;
    }

    void step(double dt_s) {
        if (dt_s <= 0.0) {
            return;
        }

        const int substeps = std::max(1, simulation_substeps_);
        const double dt_substep_s = dt_s / static_cast<double>(substeps);

        for (int i = 0; i < substeps; ++i) {
            stepSingle(dt_substep_s);
        }
    }

    void setSimulationSubsteps(int simulation_substeps) {
        simulation_substeps_ = std::max(1, simulation_substeps);
    }

    int simulationSubsteps() const {
        return simulation_substeps_;
    }

  private:
    void stepSingle(double dt_s) {
        if (dt_s <= 0.0) {
            return;
        }

        integrateRobots(dt_s);
        resolveRobotRobotImpedance();

        updateIntakeStates();

        for (std::size_t i = 0; i < balls_.size(); ++i) {
            BallEntity& ball = balls_[i];
            applyRobotCarrierPose(i, ball);
            ball.sim.step(dt_s);

            if (!ball.sim.state().held) {
                resolveRobotBallContacts(ball);
                resolveFieldElements(ball, dt_s);
                resolveFieldBounds(ball);
            } else {
                resolveFieldBounds(ball);
            }
        }
    }

    static Vector3 rotateYaw(const Vector3& local, double yaw_rad) {
        const double c = std::cos(yaw_rad);
        const double s = std::sin(yaw_rad);
        return Vector3(local.x * c - local.y * s, local.x * s + local.y * c, local.z);
    }

    static Vector3 boundaryNormalWorld(const EnvironmentalBoundary& boundary) {
        return boundary.orientation.rotate(Vector3::unitZ()).normalized();
    }

    static void applyContactImpulse(Vector3& velocity, const Vector3& normal, double restitution, double friction) {
        const double vn = velocity.dot(normal);
        if (vn >= 0.0) {
            return;
        }

        velocity -= normal * ((1.0 + std::clamp(restitution, 0.0, 1.0)) * vn);
        const Vector3 tangential = velocity - normal * velocity.dot(normal);
        velocity -= tangential * std::clamp(friction, 0.0, 1.0);
    }

    void integrateRobots(double dt_s) {
        for (auto& robot : robots_) {
            robot.position_m += robot.velocity_mps * dt_s;

            const double min_x = field_.min_corner_m.x + robot.radius_m;
            const double max_x = field_.max_corner_m.x - robot.radius_m;
            const double min_y = field_.min_corner_m.y + robot.radius_m;
            const double max_y = field_.max_corner_m.y - robot.radius_m;

            if (robot.position_m.x < min_x) {
                robot.position_m.x = min_x;
                robot.velocity_mps.x = 0.0;
            }
            if (robot.position_m.x > max_x) {
                robot.position_m.x = max_x;
                robot.velocity_mps.x = 0.0;
            }
            if (robot.position_m.y < min_y) {
                robot.position_m.y = min_y;
                robot.velocity_mps.y = 0.0;
            }
            if (robot.position_m.y > max_y) {
                robot.position_m.y = max_y;
                robot.velocity_mps.y = 0.0;
            }
        }
    }

    void resolveRobotRobotImpedance() {
        for (std::size_t i = 0; i < robots_.size(); ++i) {
            for (std::size_t j = i + 1; j < robots_.size(); ++j) {
                RobotState& a = robots_[i];
                RobotState& b = robots_[j];

                Vector3 delta = b.position_m - a.position_m;
                delta.z = 0.0;
                const double distance = delta.norm();
                const double minimum_distance = a.radius_m + b.radius_m;
                if (distance >= minimum_distance) {
                    continue;
                }

                Vector3 normal = distance > 1e-9 ? delta / distance : Vector3::unitX();
                const double overlap = minimum_distance - distance;
                a.position_m -= normal * (0.5 * overlap);
                b.position_m += normal * (0.5 * overlap);

                const double relative_speed = (a.velocity_mps - b.velocity_mps).dot(normal);
                if (relative_speed > 0.0) {
                    continue;
                }

                const double impulse = -0.7 * relative_speed;
                a.velocity_mps += normal * impulse;
                b.velocity_mps -= normal * impulse;
            }
        }
    }

    void updateIntakeStates() {
        for (auto& robot : robots_) {
            if (!robot.intake_enabled || robot.intake_capacity == 0 || robot.carried_ball_index != kNoBall) {
                continue;
            }

            const Vector3 intake_world = robot.position_m + rotateYaw(robot.intake_offset_m, robot.yaw_rad);

            std::size_t best_ball = kNoBall;
            double best_distance = std::numeric_limits<double>::infinity();

            for (std::size_t ball_index = 0; ball_index < balls_.size(); ++ball_index) {
                BallEntity& ball = balls_[ball_index];
                if (ball.sim.state().held || ball.scored_in_net) {
                    continue;
                }

                const double distance = (ball.sim.state().position_m - intake_world).norm();
                if (distance < robot.intake_radius_m && distance < best_distance) {
                    best_distance = distance;
                    best_ball = ball_index;
                }
            }

            if (best_ball == kNoBall) {
                continue;
            }

            BallEntity& selected = balls_[best_ball];
            selected.sim.setCarrierPose(robot.position_m + rotateYaw(robot.carry_offset_m, robot.yaw_rad), robot.velocity_mps);

            BallPhysicsSim3D::PickupRequest request{};
            request.intake_position_m = intake_world;
            request.capture_radius_m = robot.intake_radius_m;
            request.carry_offset_m = rotateYaw(robot.carry_offset_m, robot.yaw_rad);
            if (selected.sim.requestPickup(request)) {
                robot.carried_ball_index = best_ball;
            }
        }
    }

    void applyRobotCarrierPose(std::size_t ball_index, BallEntity& ball) {
        for (const auto& robot : robots_) {
            if (robot.carried_ball_index != ball_index) {
                continue;
            }
            const Vector3 carry_world = robot.position_m + rotateYaw(robot.carry_offset_m, robot.yaw_rad);
            ball.sim.setCarrierPose(carry_world, robot.velocity_mps);
            return;
        }
    }

    void resolveRobotBallContacts(BallEntity& ball) {
        auto state = ball.sim.state();

        for (const auto& robot : robots_) {
            Vector3 robot_to_ball = state.position_m - robot.position_m;
            robot_to_ball.z = 0.0;
            const double distance = robot_to_ball.norm();
            const double ball_radius = ball.sim.ballProperties().radius_m;
            const double contact_distance = robot.radius_m + ball_radius;
            if (distance >= contact_distance) {
                continue;
            }

            const Vector3 normal = distance > 1e-9 ? robot_to_ball / distance : Vector3::unitX();
            const double penetration = contact_distance - distance;
            state.position_m += normal * penetration;

            // Plowing behavior: transfer robot planar velocity into the ball.
            const Vector3 robot_planar_velocity(robot.velocity_mps.x, robot.velocity_mps.y, 0.0);
            state.velocity_mps.x =
                field_.plow_ball_velocity_retention * state.velocity_mps.x +
                field_.plow_robot_velocity_gain * robot_planar_velocity.x;
            state.velocity_mps.y =
                field_.plow_ball_velocity_retention * state.velocity_mps.y +
                field_.plow_robot_velocity_gain * robot_planar_velocity.y;

            applyContactImpulse(state.velocity_mps,
                                normal,
                                field_.robot_ball_contact_restitution,
                                field_.robot_ball_contact_friction);
        }

        ball.sim.setState(state);
    }

    void resolveFieldBounds(BallEntity& ball) const {
        auto state = ball.sim.state();
        const double radius = ball.sim.ballProperties().radius_m;

        const double min_x = field_.min_corner_m.x + radius;
        const double max_x = field_.max_corner_m.x - radius;
        const double min_y = field_.min_corner_m.y + radius;
        const double max_y = field_.max_corner_m.y - radius;

        if (state.position_m.x < min_x) {
            state.position_m.x = min_x;
            state.velocity_mps.x = std::abs(state.velocity_mps.x) * field_.wall_restitution;
            state.velocity_mps.y *= (1.0 - field_.wall_friction);
        }
        if (state.position_m.x > max_x) {
            state.position_m.x = max_x;
            state.velocity_mps.x = -std::abs(state.velocity_mps.x) * field_.wall_restitution;
            state.velocity_mps.y *= (1.0 - field_.wall_friction);
        }
        if (state.position_m.y < min_y) {
            state.position_m.y = min_y;
            state.velocity_mps.y = std::abs(state.velocity_mps.y) * field_.wall_restitution;
            state.velocity_mps.x *= (1.0 - field_.wall_friction);
        }
        if (state.position_m.y > max_y) {
            state.position_m.y = max_y;
            state.velocity_mps.y = -std::abs(state.velocity_mps.y) * field_.wall_restitution;
            state.velocity_mps.x *= (1.0 - field_.wall_friction);
        }

        ball.sim.setState(state);
    }

    void resolveFieldElements(BallEntity& ball, double dt_s) {
        auto state = ball.sim.state();
        const double radius = ball.sim.ballProperties().radius_m;

        for (const auto& boundary : field_elements_) {
            if (!boundary.is_active) {
                continue;
            }

            if (field_.net_boundary_user_id >= 0 && boundary.user_id == field_.net_boundary_user_id &&
                boundary.type == BoundaryType::kBox) {
                if (isInsideBoxBoundary(state.position_m, boundary, radius * field_.net_entry_slack_scale)) {
                    ball.scored_in_net = true;
                    state.velocity_mps.x *= field_.net_velocity_decay;
                    state.velocity_mps.y *= field_.net_velocity_decay;
                    state.velocity_mps.z = std::min(state.velocity_mps.z, 0.0);
                    state.spin_radps *= field_.net_spin_decay;
                    state.velocity_mps += Vector3(0.0, 0.0, -field_.net_downward_bias_mps2 * dt_s);
                }
                continue;
            }

            switch (boundary.type) {
                case BoundaryType::kPlane:
                case BoundaryType::kWall:
                    resolvePlaneBoundary(state, boundary, radius);
                    break;
                case BoundaryType::kBox:
                    resolveBoxBoundary(state, boundary, radius);
                    break;
                case BoundaryType::kCylinder:
                    resolveCylinderBoundary(state, boundary, radius);
                    break;
                default:
                    break;
            }
        }

        ball.sim.setState(state);
    }

    static bool isInsideBoxBoundary(const Vector3& world_point, const EnvironmentalBoundary& boundary,
                                    double slack) {
        const Quaternion inverse = boundary.orientation.inverse();
        const Vector3 local = inverse.rotate(world_point - boundary.position_m);
        return std::abs(local.x) <= boundary.half_extents_m.x + slack &&
               std::abs(local.y) <= boundary.half_extents_m.y + slack &&
               std::abs(local.z) <= boundary.half_extents_m.z + slack;
    }

    static void resolvePlaneBoundary(BallPhysicsSim3D::BallState& state, const EnvironmentalBoundary& boundary,
                                     double ball_radius) {
        Vector3 normal = boundaryNormalWorld(boundary);
        if (normal.isZero()) {
            normal = Vector3::unitZ();
        }

        const double signed_distance = (state.position_m - boundary.position_m).dot(normal);
        if (signed_distance >= ball_radius) {
            return;
        }

        state.position_m += normal * (ball_radius - signed_distance);
        applyContactImpulse(state.velocity_mps, normal, boundary.restitution, boundary.friction_coefficient);
    }

    static void resolveBoxBoundary(BallPhysicsSim3D::BallState& state, const EnvironmentalBoundary& boundary,
                                   double ball_radius) {
        const Quaternion inverse = boundary.orientation.inverse();
        const Vector3 local = inverse.rotate(state.position_m - boundary.position_m);

        const Vector3 closest(
            std::clamp(local.x, -boundary.half_extents_m.x, boundary.half_extents_m.x),
            std::clamp(local.y, -boundary.half_extents_m.y, boundary.half_extents_m.y),
            std::clamp(local.z, -boundary.half_extents_m.z, boundary.half_extents_m.z));

        Vector3 delta = local - closest;
        double distance = delta.norm();
        Vector3 local_normal = distance > 1e-9 ? delta / distance : Vector3::unitZ();

        if (distance >= ball_radius) {
            return;
        }

        if (distance <= 1e-9) {
            const double pen_x = boundary.half_extents_m.x - std::abs(local.x);
            const double pen_y = boundary.half_extents_m.y - std::abs(local.y);
            const double pen_z = boundary.half_extents_m.z - std::abs(local.z);

            if (pen_x <= pen_y && pen_x <= pen_z) {
                local_normal = Vector3(local.x >= 0.0 ? 1.0 : -1.0, 0.0, 0.0);
                distance = pen_x;
            } else if (pen_y <= pen_z) {
                local_normal = Vector3(0.0, local.y >= 0.0 ? 1.0 : -1.0, 0.0);
                distance = pen_y;
            } else {
                local_normal = Vector3(0.0, 0.0, local.z >= 0.0 ? 1.0 : -1.0);
                distance = pen_z;
            }
        }

        const Vector3 world_normal = boundary.orientation.rotate(local_normal).normalized();
        state.position_m += world_normal * (ball_radius - distance);
        applyContactImpulse(state.velocity_mps, world_normal, boundary.restitution, boundary.friction_coefficient);
    }

    static void resolveCylinderBoundary(BallPhysicsSim3D::BallState& state, const EnvironmentalBoundary& boundary,
                                        double ball_radius) {
        const Quaternion inverse = boundary.orientation.inverse();
        const Vector3 local = inverse.rotate(state.position_m - boundary.position_m);

        const double half_height = std::max(0.0, boundary.half_extents_m.z);
        if (std::abs(local.z) > half_height + ball_radius) {
            return;
        }

        const Vector3 radial(local.x, local.y, 0.0);
        const double radial_distance = radial.norm();
        const double contact_distance = std::max(0.0, boundary.radius_m) + ball_radius;
        if (radial_distance >= contact_distance) {
            return;
        }

        const Vector3 radial_normal_local = radial_distance > 1e-9 ? radial / radial_distance : Vector3::unitX();
        const Vector3 radial_normal_world = boundary.orientation.rotate(radial_normal_local).normalized();
        state.position_m += radial_normal_world * (contact_distance - radial_distance);
        applyContactImpulse(state.velocity_mps, radial_normal_world, boundary.restitution, boundary.friction_coefficient);
    }

    FieldConfig field_{};
    std::vector<RobotState> robots_{};
    std::vector<BallEntity> balls_{};
    std::vector<EnvironmentalBoundary> field_elements_{};
    int simulation_substeps_{4};
};

}  // namespace frcsim
