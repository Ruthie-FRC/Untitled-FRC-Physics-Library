#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "frcsim/field/boundary.hpp"
#include "frcsim/gamepiece/ball_physics.hpp"
#include "frcsim/math/quaternion.hpp"
#include "frcsim/math/vector.hpp"

namespace frcsim {

/**
 * @brief Simulates robot interaction with spherical game pieces and free-flight projectiles.
 *
 * The simulator owns robot, grounded ball, projectile, goal-zone, and field-boundary state.
 * Seasonal presets are intentionally provided outside this class so this type stays reusable.
 */
class BallGamepieceSim {
  public:
    /** Sentinel index representing no carried ball. */
    static constexpr std::size_t kNoBall = static_cast<std::size_t>(-1);

    /** @brief Global field and contact tuning parameters. */
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

    /** @brief Per-robot state used by game piece interaction and collision routines. */
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

    /**
     * @brief Exit trajectory and metadata used when launching a carried ball or a projectile.
     */
    struct ExitTrajectoryParameters {
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

        std::string gamepiece_type{"Ball"};
    };

    /** @brief Backward-compatible alias for older call sites. */
    using FireCommand = ExitTrajectoryParameters;

    /** @brief Registration record for a named game piece type. */
    struct GamePieceInfo {
        std::string type{"Ball"};
        BallPhysicsSim3D::Config physics_config{};
        BallPhysicsSim3D::BallProperties ball_properties{};
        bool spawn_on_ground_after_projectile{true};
    };

    /** @brief In-flight entity not currently represented by BallPhysicsSim3D. */
    struct ProjectileEntity {
        std::string type{"Ball"};
        Vector3 position_m{};
        Vector3 velocity_mps{};
        Vector3 spin_radps{};
        double gravity_mps2{9.81};
        double age_s{0.0};
        bool active{true};
        bool spawn_on_ground_after_touch{true};
        bool hit_target{false};
        std::function<void()> hit_target_callback{};
    };

    /** @brief Goal capture region and validation logic. */
    struct GoalZone {
        enum class Shape {
            kBox,
            kSphere,
        };

        Shape shape{Shape::kBox};
        Vector3 center_m{};
        Vector3 half_extents_m{0.2, 0.2, 0.2};
        double radius_m{0.25};
        std::string accepted_type{"Ball"};
        bool require_positive_vertical_velocity{false};
        std::function<bool(const Vector3&)> custom_velocity_validator{};
    };

    /** @brief Grounded game piece entity with its own ball physics instance. */
    struct BallEntity {
        BallPhysicsSim3D sim{};
        bool scored_in_net{false};
    };

    /** @brief Constructs a simulator with default evergreen field bounds and contact tuning. */
    BallGamepieceSim() = default;

    /** @brief Constructs a simulator with the provided field configuration. */
    explicit BallGamepieceSim(const FieldConfig& field) : field_(field) {}

    /** @brief Returns baseline evergreen field bounds and wall-contact coefficients. */
    static FieldConfig evergreenFieldConfig() {
        return FieldConfig{};
    }

    /** @brief Replaces field configuration while preserving dynamic simulation entities. */
    void setFieldConfig(const FieldConfig& field) {
        field_ = field;
    }

    using RobotAddedCallback = std::function<void(std::size_t robot_index, const RobotState& robot)>;

    /**
     * @brief Adds a robot and returns its index.
     *
     * If a robot-added callback is configured, it is invoked after insertion.
     */
    std::size_t addRobot(const RobotState& robot) {
        robots_.push_back(robot);
        const std::size_t robot_index = robots_.size() - 1;
        if (robot_added_callback_) {
            robot_added_callback_(robot_index, robots_[robot_index]);
        }
        return robot_index;
    }

    /** @brief Sets a callback invoked whenever addRobot inserts a robot. */
    void setRobotAddedCallback(const RobotAddedCallback& callback) {
        robot_added_callback_ = callback;
    }

    /**
     * @brief Adds a grounded ball with explicit physics configuration.
     * @return Index of the inserted ball entity.
     */
    std::size_t addBall(const BallPhysicsSim3D::BallState& state,
                        const BallPhysicsSim3D::Config& config,
                        const BallPhysicsSim3D::BallProperties& properties) {
        BallEntity entity{};
        entity.sim = BallPhysicsSim3D(config, properties);
        entity.sim.setState(state);
        balls_.push_back(entity);
        ball_types_.push_back("Ball");
        return balls_.size() - 1;
    }

    /** @brief Adds a projectile entity and returns its index. */
    std::size_t addProjectile(const ProjectileEntity& projectile) {
        projectiles_.push_back(projectile);
        return projectiles_.size() - 1;
    }

    /** @brief Adds a goal zone and returns a mutable reference to the stored copy. */
    GoalZone& addGoalZone(const GoalZone& goal_zone) {
        goals_.push_back(goal_zone);
        return goals_.back();
    }

    /**
     * @brief Registers or replaces a named game piece type.
     * @return Mutable reference to stored registration.
     */
    GamePieceInfo& registerGamePieceType(const GamePieceInfo& info) {
        for (auto& existing : gamepiece_types_) {
            if (existing.type == info.type) {
                existing = info;
                return existing;
            }
        }
        gamepiece_types_.push_back(info);
        return gamepiece_types_.back();
    }

    /** @brief Clears all registered game piece type definitions. */
    void clearGamePieceTypes() {
        gamepiece_types_.clear();
    }

    /** @brief Applies evergreen field configuration and clears dynamic field elements/goals. */
    void configureEvergreenField() {
        field_ = evergreenFieldConfig();
        field_elements_.clear();
        goals_.clear();
    }

    /** @brief Adds a field element and returns a mutable reference to the stored copy. */
    EnvironmentalBoundary& addFieldElement(const EnvironmentalBoundary& boundary) {
        field_elements_.push_back(boundary);
        return field_elements_.back();
    }

    /** @brief Mutable robot state list. */
    std::vector<RobotState>& robots() { return robots_; }
    /** @brief Immutable robot state list. */
    const std::vector<RobotState>& robots() const { return robots_; }

    /** @brief Mutable grounded-ball list. */
    std::vector<BallEntity>& balls() { return balls_; }
    /** @brief Immutable grounded-ball list. */
    const std::vector<BallEntity>& balls() const { return balls_; }

    /** @brief Returns the registered type string for a ball index, or empty string if out of range. */
    const std::string& ballType(std::size_t ball_index) const {
        static const std::string empty;
        if (ball_index >= ball_types_.size()) {
            return empty;
        }
        return ball_types_[ball_index];
    }

    /**
     * @brief Updates the type label for an existing ball.
     * @return true if the ball index was valid and updated.
     */
    bool setBallType(std::size_t ball_index, const std::string& type) {
        if (ball_index >= ball_types_.size()) {
            return false;
        }
        ball_types_[ball_index] = type;
        return true;
    }

    /**
     * @brief Removes a ball by index and remaps carried-ball indices accordingly.
     * @return true when removal succeeded.
     */
    bool removeBall(std::size_t ball_index) {
        if (ball_index >= balls_.size() || ball_index >= ball_types_.size()) {
            return false;
        }

        balls_.erase(balls_.begin() + static_cast<std::ptrdiff_t>(ball_index));
        ball_types_.erase(ball_types_.begin() + static_cast<std::ptrdiff_t>(ball_index));

        for (auto& robot : robots_) {
            if (robot.carried_ball_index == ball_index) {
                robot.carried_ball_index = kNoBall;
            } else if (robot.carried_ball_index != kNoBall && robot.carried_ball_index > ball_index) {
                --robot.carried_ball_index;
            }
        }
        return true;
    }

    /** @brief Mutable projectile list. */
    std::vector<ProjectileEntity>& projectiles() { return projectiles_; }
    /** @brief Immutable projectile list. */
    const std::vector<ProjectileEntity>& projectiles() const { return projectiles_; }

    /** @brief Mutable goal-zone list. */
    std::vector<GoalZone>& goals() { return goals_; }
    /** @brief Immutable goal-zone list. */
    const std::vector<GoalZone>& goals() const { return goals_; }

    /** @brief Mutable field-element list. */
    std::vector<EnvironmentalBoundary>& fieldElements() { return field_elements_; }
    /** @brief Immutable field-element list. */
    const std::vector<EnvironmentalBoundary>& fieldElements() const { return field_elements_; }

    /** @brief Returns total grounded ball count (including scored-in-net balls). */
    std::size_t countBalls() const { return balls_.size(); }

    /** @brief Returns count of currently active projectiles. */
    std::size_t countProjectiles() const {
        std::size_t count = 0;
        for (const auto& projectile : projectiles_) {
            if (projectile.active) {
                ++count;
            }
        }
        return count;
    }

    /** @brief Returns count of grounded balls marked as scored in configured net volume(s). */
    std::size_t countScoredBalls() const {
        std::size_t count = 0;
        for (const auto& ball : balls_) {
            if (ball.scored_in_net) {
                ++count;
            }
        }
        return count;
    }

    /**
     * @brief Launches the robot's carried ball using exit trajectory parameters.
     * @return true if a carried ball existed and was launched.
     */
    bool fireBall(std::size_t robot_index, const ExitTrajectoryParameters& command) {
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

        if (!ball_types_.empty()) {
            const std::size_t ball_index = static_cast<std::size_t>(&ball - &balls_[0]);
            if (ball_index < ball_types_.size()) {
                ball_types_[ball_index] = command.gamepiece_type;
            }
        }
        return true;
    }

    /**
     * @brief Spawns a projectile using robot pose and exit trajectory parameters.
     * @return Projectile index, or kNoBall when robot index is invalid.
     */
    std::size_t fireProjectile(std::size_t robot_index, const ExitTrajectoryParameters& command,
                               bool spawn_on_ground_after_touch = true,
                               const std::function<void()>& hit_target_callback = {}) {
        if (robot_index >= robots_.size()) {
            return kNoBall;
        }

        const RobotState& robot = robots_[robot_index];
        const double launch_speed = command.estimated_exit_velocity_mps > 0.0
                                        ? command.estimated_exit_velocity_mps
                                        : std::max(0.0, command.mechanism_speed_mps);

        const double yaw_world = robot.yaw_rad + command.yaw_offset_rad;
        const double cos_pitch = std::cos(command.pitch_rad);
        const Vector3 direction_world(std::cos(yaw_world) * cos_pitch,
                                      std::sin(yaw_world) * cos_pitch,
                                      std::sin(command.pitch_rad));

        ProjectileEntity projectile{};
        projectile.type = command.gamepiece_type;
        projectile.position_m = robot.position_m + rotateYaw(command.launch_offset_m, robot.yaw_rad);
        projectile.velocity_mps = robot.velocity_mps + direction_world * launch_speed;
        projectile.spin_radps = command.spin_radps;
        projectile.gravity_mps2 = std::max(0.0, -fallbackBallConfig().gravity_mps2.z);
        projectile.spawn_on_ground_after_touch = spawn_on_ground_after_touch;
        projectile.hit_target_callback = hit_target_callback;

        projectiles_.push_back(projectile);
        return projectiles_.size() - 1;
    }

    /** @brief Advances simulation by dt using configured internal substep count. */
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

    /** @brief Sets simulation substeps per external step; values below 1 clamp to 1. */
    void setSimulationSubsteps(int simulation_substeps) {
        simulation_substeps_ = std::max(1, simulation_substeps);
    }

    /** @brief Returns current simulation substeps-per-step value. */
    int simulationSubsteps() const {
        return simulation_substeps_;
    }

  private:
        /** @brief Executes one fixed simulation substep. */
    void stepSingle(double dt_s) {
        if (dt_s <= 0.0) {
            return;
        }

        integrateRobots(dt_s);
        resolveRobotRobotImpedance();
        updateProjectiles(dt_s);

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

    void updateProjectiles(double dt_s) {
        const double floor_z = fallbackBallConfig().ground_height_m;

        for (auto& projectile : projectiles_) {
            if (!projectile.active) {
                continue;
            }

            projectile.age_s += dt_s;
            projectile.velocity_mps.z -= projectile.gravity_mps2 * dt_s;
            projectile.position_m += projectile.velocity_mps * dt_s;

            if (checkProjectileGoalHit(projectile)) {
                projectile.hit_target = true;
                projectile.active = false;
                if (projectile.hit_target_callback) {
                    projectile.hit_target_callback();
                }
                continue;
            }

            if (projectile.position_m.z <= floor_z) {
                if (projectile.spawn_on_ground_after_touch) {
                    const GamePieceInfo* info = findGamePieceInfo(projectile.type);
                    const auto& physics_cfg = (info != nullptr) ? info->physics_config : fallbackBallConfig();
                    const auto& physics_props = (info != nullptr) ? info->ball_properties : fallbackBallProperties();

                    BallPhysicsSim3D::BallState state{};
                    state.position_m = Vector3(projectile.position_m.x, projectile.position_m.y,
                                               std::max(physics_props.radius_m, physics_cfg.ground_height_m));
                    state.velocity_mps = projectile.velocity_mps;
                    state.spin_radps = projectile.spin_radps;
                    addBall(state, physics_cfg, physics_props);
                    if (!ball_types_.empty()) {
                        ball_types_.back() = projectile.type;
                    }
                }
                projectile.active = false;
                continue;
            }

            if (isProjectileOutOfField(projectile)) {
                projectile.active = false;
            }
        }
    }

    bool checkProjectileGoalHit(const ProjectileEntity& projectile) const {
        for (const auto& goal : goals_) {
            if (!goal.accepted_type.empty() && goal.accepted_type != projectile.type) {
                continue;
            }
            if (goal.require_positive_vertical_velocity && projectile.velocity_mps.z <= 0.0) {
                continue;
            }
            if (goal.custom_velocity_validator && !goal.custom_velocity_validator(projectile.velocity_mps)) {
                continue;
            }

            if (goal.shape == GoalZone::Shape::kBox) {
                const Vector3 delta = projectile.position_m - goal.center_m;
                if (std::abs(delta.x) <= goal.half_extents_m.x &&
                    std::abs(delta.y) <= goal.half_extents_m.y &&
                    std::abs(delta.z) <= goal.half_extents_m.z) {
                    return true;
                }
            } else {
                if ((projectile.position_m - goal.center_m).norm() <= goal.radius_m) {
                    return true;
                }
            }
        }
        return false;
    }

    bool isProjectileOutOfField(const ProjectileEntity& projectile) const {
        const double tolerance_m = 2.0;
        return projectile.position_m.x < field_.min_corner_m.x - tolerance_m ||
               projectile.position_m.x > field_.max_corner_m.x + tolerance_m ||
               projectile.position_m.y < field_.min_corner_m.y - tolerance_m ||
               projectile.position_m.y > field_.max_corner_m.y + tolerance_m;
    }

    const GamePieceInfo* findGamePieceInfo(const std::string& type) const {
        for (const auto& info : gamepiece_types_) {
            if (info.type == type) {
                return &info;
            }
        }
        return nullptr;
    }

    static BallPhysicsSim3D::BallProperties fallbackBallProperties() {
        BallPhysicsSim3D::BallProperties properties{};
        properties.mass_kg = 0.24;
        properties.radius_m = 0.09;
        properties.drag_coefficient = 0.50;
        properties.reference_area_m2 = 3.14159265358979323846 * properties.radius_m * properties.radius_m;
        properties.restitution = 0.48;
        return properties;
    }

    static BallPhysicsSim3D::Config fallbackBallConfig() {
        BallPhysicsSim3D::Config config{};
        config.gravity_mps2 = Vector3(0.0, 0.0, -9.81);
        config.air_density_kgpm3 = 1.225;
        config.magnus_coefficient = 1.0e-4;
        config.ground_height_m = 0.0;
        config.rolling_friction_per_s = 1.5;
        config.min_bounce_speed_mps = 0.06;
        return config;
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
    std::vector<std::string> ball_types_{};
    std::vector<ProjectileEntity> projectiles_{};
    std::vector<GoalZone> goals_{};
    std::vector<GamePieceInfo> gamepiece_types_{};
    std::vector<EnvironmentalBoundary> field_elements_{};
    RobotAddedCallback robot_added_callback_{};
    int simulation_substeps_{4};
};

}  // namespace frcsim
