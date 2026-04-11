// Copyright (c) Jsim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

#include <algorithm>
#include <functional>
#include <vector>

#include "frcsim/field/goal_structure.hpp"
#include "frcsim/field/obstacle.hpp"
#include "frcsim/gamepiece/ball_gamepiece_sim.hpp"
#include "frcsim/gamepiece/intake_simulation.hpp"

namespace frcsim {

/**
 * @brief High-level arena wrapper that coordinates game piece simulation,
 * intake helpers, and custom subtick hooks.
 */
class SimulatedArena {
 public:
  /** @brief Loop timing parameters for robot-period and internal simulation
   * subticks. */
  struct Timings {
    /** External control-period duration in seconds. */
    double robot_period_s{0.02};
    /** Number of simulation updates performed inside each robot period. */
    int simulation_subticks_per_period{5};
  };

  /** @brief Convenience field-description bundle converted into
   * BallGamepieceSim entities. */
  struct FieldMap {
    /** Obstacles converted to field boundaries. */
    std::vector<FieldObstacle> obstacles{};
    /** Scoring regions converted to goal zones. */
    std::vector<GoalStructure> goals{};

    /**
     * @brief Applies map content into the provided game-piece simulator
     * instance.
     * @param sim Simulator to receive converted obstacles/goals.
     */
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
        switch (goal.accepted_type) {
          case GoalStructure::AcceptedType::kAny:
            zone.accept_any_type = true;
            zone.accepted_type = BallGamepieceSim::GamePieceType::kBall;
            break;
          case GoalStructure::AcceptedType::kBall:
            zone.accepted_type = BallGamepieceSim::GamePieceType::kBall;
            break;
          case GoalStructure::AcceptedType::kCustom1:
            zone.accepted_type = BallGamepieceSim::GamePieceType::kCustom1;
            break;
          case GoalStructure::AcceptedType::kCustom2:
            zone.accepted_type = BallGamepieceSim::GamePieceType::kCustom2;
            break;
          case GoalStructure::AcceptedType::kCustom3:
            zone.accepted_type = BallGamepieceSim::GamePieceType::kCustom3;
            break;
          case GoalStructure::AcceptedType::kCustom4:
            zone.accepted_type = BallGamepieceSim::GamePieceType::kCustom4;
            break;
        }
        zone.require_positive_vertical_velocity =
            goal.require_positive_vertical_velocity;
        zone.custom_velocity_validator = goal.custom_velocity_validator;
        sim.addGoalZone(zone);
      }
    }
  };

  using CustomSimulation =
      std::function<void(int sub_tick_index, SimulatedArena&)>;
  using RobotRegisteredCallback =
      std::function<void(std::size_t robot_index,
                         const BallGamepieceSim::RobotState&, SimulatedArena&)>;

  /** @brief Constructs arena with default field config and callback plumbing.
   */
  SimulatedArena() {
    gamepiece_sim_.setRobotAddedCallback(
        [this](std::size_t robot_index,
               const BallGamepieceSim::RobotState& robot) {
          if (robot_registered_callback_) {
            robot_registered_callback_(robot_index, robot, *this);
          }
        });
  }

  /**
   * @brief Constructs arena with explicit field configuration and callback
   * plumbing.
   * @param field_config Initial field configuration for the internal gamepiece
   * simulator.
   */
  explicit SimulatedArena(const BallGamepieceSim::FieldConfig& field_config)
      : gamepiece_sim_(field_config) {
    gamepiece_sim_.setRobotAddedCallback(
        [this](std::size_t robot_index,
               const BallGamepieceSim::RobotState& robot) {
          if (robot_registered_callback_) {
            robot_registered_callback_(robot_index, robot, *this);
          }
        });
  }

  /**
   * @brief Mutable access to underlying game-piece simulator.
   * @return Mutable simulator reference.
   */
  BallGamepieceSim& gamepieceSim() { return gamepiece_sim_; }
  /**
   * @brief Immutable access to underlying game-piece simulator.
   * @return Const simulator reference.
   */
  const BallGamepieceSim& gamepieceSim() const { return gamepiece_sim_; }

  /**
   * @brief Adds a robot to the underlying game-piece simulator.
   * @param robot Robot state to insert.
   * @return Inserted robot index.
   */
  std::size_t addRobot(const BallGamepieceSim::RobotState& robot) {
    return gamepiece_sim_.addRobot(robot);
  }

  /**
   * @brief Sets callback fired after a robot is registered through this arena.
   * @param callback Callback to invoke on robot registration.
   */
  void setRobotRegisteredCallback(const RobotRegisteredCallback& callback) {
    robot_registered_callback_ = callback;
  }

  /**
   * @brief Updates timing parameters with safety clamps for minimum valid
   * values.
   * @param timings New timing values.
   */
  void setTimings(const Timings& timings) {
    timings_.robot_period_s = std::max(1e-6, timings.robot_period_s);
    timings_.simulation_subticks_per_period =
        std::max(1, timings.simulation_subticks_per_period);
  }

  /**
   * @brief Returns active timing parameters.
   * @return Immutable Timings reference.
   */
  const Timings& timings() const { return timings_; }

  /**
   * @brief Applies obstacles and goals from a field-map bundle.
   * @param field_map Map to apply.
   */
  void applyFieldMap(const FieldMap& field_map) {
    field_map.applyTo(gamepiece_sim_);
  }

  /**
   * @brief Adds a custom per-subtick callback if callable is non-empty.
   * @param custom_simulation Callback to append.
   */
  void addCustomSimulation(const CustomSimulation& custom_simulation) {
    if (custom_simulation) {
      custom_simulations_.push_back(custom_simulation);
    }
  }

  /**
   * @brief Creates and stores an intake simulation helper from config.
   * @param config Intake simulation configuration.
   * @return Mutable reference to inserted intake simulation.
   */
  IntakeSimulation& addIntakeSimulation(
      const IntakeSimulation::Config& config) {
    intake_simulations_.emplace_back(config);
    return intake_simulations_.back();
  }

  /**
   * @brief Mutable intake simulation list.
   * @return Mutable list reference.
   */
  std::vector<IntakeSimulation>& intakeSimulations() {
    return intake_simulations_;
  }
  /**
   * @brief Immutable intake simulation list.
   * @return Const list reference.
   */
  const std::vector<IntakeSimulation>& intakeSimulations() const {
    return intake_simulations_;
  }

  /**
   * @brief Executes one robot-period worth of simulation.
   *
   * Per subtick order: gamepiece physics, intake updates, custom callbacks.
   */
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
