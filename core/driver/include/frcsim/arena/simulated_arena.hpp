#pragma once

#include <algorithm>
#include <functional>
#include <vector>

#include "frcsim/field/goal_structure.hpp"
#include "frcsim/field/obstacle.hpp"
#include "frcsim/gamepiece/ball_gamepiece_sim.hpp"
#include "frcsim/gamepiece/intake_simulation.hpp"

namespace frcsim {

class SimulatedArena {
  public:
    struct Timings {
        double robot_period_s{0.02};
        int simulation_subticks_per_period{5};
    };

    struct FieldMap {
        std::vector<FieldObstacle> obstacles{};
        std::vector<GoalStructure> goals{};

        void applyTo(BallGamepieceSim& sim) const {
            for (const auto& obstacle : obstacles) {
                sim.addFieldElement(obstacle.boundary);
            }
            for (const auto& goal : goals) {
                BallGamepieceSim::GoalZone zone{};
                zone.shape = (goal.shape == GoalStructure::Shape::kSphere)
                                 ? BallGamepieceSim::GoalZone::Shape::kSphere
                                 : BallGamepieceSim::GoalZone::Shape::kBox;
                zone.center_m = goal.center_m;
                zone.half_extents_m = goal.half_extents_m;
                zone.radius_m = goal.radius_m;
                zone.accepted_type = goal.accepted_type;
                zone.require_positive_vertical_velocity = goal.require_positive_vertical_velocity;
                zone.custom_velocity_validator = goal.custom_velocity_validator;
                sim.addGoalZone(zone);
            }
        }
    };

    using CustomSimulation = std::function<void(int sub_tick_index, SimulatedArena&)>;
    using RobotRegisteredCallback =
        std::function<void(std::size_t robot_index, const BallGamepieceSim::RobotState&, SimulatedArena&)>;

    SimulatedArena() {
        gamepiece_sim_.setRobotAddedCallback(
            [this](std::size_t robot_index, const BallGamepieceSim::RobotState& robot) {
                if (robot_registered_callback_) {
                    robot_registered_callback_(robot_index, robot, *this);
                }
            });
    }

    explicit SimulatedArena(const BallGamepieceSim::FieldConfig& field_config)
        : gamepiece_sim_(field_config) {
        gamepiece_sim_.setRobotAddedCallback(
            [this](std::size_t robot_index, const BallGamepieceSim::RobotState& robot) {
                if (robot_registered_callback_) {
                    robot_registered_callback_(robot_index, robot, *this);
                }
            });
    }

    BallGamepieceSim& gamepieceSim() { return gamepiece_sim_; }
    const BallGamepieceSim& gamepieceSim() const { return gamepiece_sim_; }

    std::size_t addRobot(const BallGamepieceSim::RobotState& robot) {
        return gamepiece_sim_.addRobot(robot);
    }

    void setRobotRegisteredCallback(const RobotRegisteredCallback& callback) {
        robot_registered_callback_ = callback;
    }

    void setTimings(const Timings& timings) {
        timings_.robot_period_s = std::max(1e-6, timings.robot_period_s);
        timings_.simulation_subticks_per_period = std::max(1, timings.simulation_subticks_per_period);
    }

    const Timings& timings() const { return timings_; }

    void applyFieldMap(const FieldMap& field_map) {
        field_map.applyTo(gamepiece_sim_);
    }

    void addCustomSimulation(const CustomSimulation& custom_simulation) {
        if (custom_simulation) {
            custom_simulations_.push_back(custom_simulation);
        }
    }

    IntakeSimulation& addIntakeSimulation(const IntakeSimulation::Config& config) {
        intake_simulations_.emplace_back(config);
        return intake_simulations_.back();
    }

    std::vector<IntakeSimulation>& intakeSimulations() { return intake_simulations_; }
    const std::vector<IntakeSimulation>& intakeSimulations() const { return intake_simulations_; }

    void simulationPeriodic() {
        const int subticks = std::max(1, timings_.simulation_subticks_per_period);
        const double dt = timings_.robot_period_s / static_cast<double>(subticks);

        for (int sub_tick = 0; sub_tick < subticks; ++sub_tick) {
            gamepiece_sim_.step(dt);

            for (auto& intake : intake_simulations_) {
                intake.update(gamepiece_sim_);
            }

            for (auto& custom_sim : custom_simulations_) {
                custom_sim(sub_tick, *this);
            }
        }
    }

  private:
    Timings timings_{};
    BallGamepieceSim gamepiece_sim_{};
    std::vector<IntakeSimulation> intake_simulations_{};
    std::vector<CustomSimulation> custom_simulations_{};
        RobotRegisteredCallback robot_registered_callback_{};
};

}  // namespace frcsim
