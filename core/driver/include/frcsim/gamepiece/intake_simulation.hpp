// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

#include <cstddef>
#include <deque>
#include <functional>
#include <string>

#include "frcsim/gamepiece/ball_gamepiece_sim.hpp"

namespace frcsim {

/** @brief Lightweight intake interaction model that consumes nearby balls into
 * robot inventory. */
class IntakeSimulation {
 public:
  /** @brief Runtime configuration for intake behavior. */
  struct Config {
    /** Robot index in BallGamepieceSim::robots(). */
    std::size_t robot_index{0};
    /** Type string filter; empty matches any type. */
    std::string targeted_type{"Ball"};
    /** Maximum number of stored pieces in this intake simulation. */
    std::size_t capacity{1};
  };

  /** @brief Contact event generated during update() proximity checks. */
  struct ContactEvent {
    /** @brief Contact lifecycle phase. */
    enum class Phase {
      kBegin,
      kPersist,
      kEnd,
    };

    /** Index of ball involved in this event. */
    std::size_t ball_index{BallGamepieceSim::kNoBall};
    /** Contact phase classification. */
    Phase phase{Phase::kBegin};
  };

  IntakeSimulation() = default;

  /**
   * @brief Constructs intake simulation with explicit config.
   * @param config Intake configuration to store.
   */
  explicit IntakeSimulation(const Config& config) : config_(config) {}

  /** @brief Replaces intake configuration. @param config New configuration. */
  void setConfig(const Config& config) { config_ = config; }
  /** @brief Returns current configuration. @return Immutable config reference.
   */
  const Config& config() const { return config_; }

  /** @brief Enables/disables intake processing. @param running True to enable
   * update processing. */
  void setRunning(bool running) { running_ = running; }
  /** @brief Returns whether intake processing is active. @return True if
   * running. */
  bool isRunning() const { return running_; }

  /**
   * @brief Sets optional custom acceptance predicate for candidate balls.
   * @param predicate Callable receiving ball index and simulator state.
   */
  void setCustomIntakeCondition(
      const std::function<bool(std::size_t, const BallGamepieceSim&)>&
          predicate) {
    custom_condition_ = predicate;
  }

  /** @brief Returns current intake inventory count. @return Number of stored
   * pieces. */
  std::size_t gamePiecesInIntakeCount() const { return intake_count_; }

  /**
   * @brief Removes one stored piece from intake inventory.
   * @return True when a piece was available and consumed.
   */
  bool obtainGamePieceFromIntake() {
    if (intake_count_ == 0) {
      return false;
    }
    --intake_count_;
    return true;
  }

  /** @brief Returns contact events generated during the most recent update
   * call. @return Event deque. */
  const std::deque<ContactEvent>& recentEvents() const {
    return recent_events_;
  }

  /**
   * @brief Performs proximity scanning and intake processing for one simulation
   * step.
   * @param sim Mutable gamepiece simulator state.
   */
  void update(BallGamepieceSim& sim) {
    if (!running_) {
      recent_events_.clear();
      return;
    }
    if (config_.robot_index >= sim.robots().size()) {
      recent_events_.clear();
      return;
    }

    const auto& robot = sim.robots()[config_.robot_index];
    const Vector3 intake_world = robot.position_m;

    recent_events_.clear();
    for (std::size_t i = 0; i < sim.balls().size(); ++i) {
      const auto& ball = sim.balls()[i];
      if (ball.sim.state().held) {
        continue;
      }
      if (!config_.targeted_type.empty() &&
          sim.ballTypeName(i) != config_.targeted_type) {
        continue;
      }

      const double distance =
          (ball.sim.state().position_m - intake_world).norm();
      if (distance <= robot.intake_radius_m) {
        recent_events_.push_back({i, ContactEvent::Phase::kBegin});
      }
    }

    processIntakeQueue(sim);
  }

 private:
  void processIntakeQueue(BallGamepieceSim& sim) {
    if (intake_count_ >= config_.capacity) {
      return;
    }

    for (const auto& event : recent_events_) {
      if (event.phase != ContactEvent::Phase::kBegin) {
        continue;
      }
      if (intake_count_ >= config_.capacity) {
        break;
      }
      if (event.ball_index >= sim.balls().size()) {
        continue;
      }
      if (custom_condition_ && !custom_condition_(event.ball_index, sim)) {
        continue;
      }
      if (sim.removeBall(event.ball_index)) {
        ++intake_count_;
      }
    }
  }

  Config config_{};
  bool running_{false};
  std::size_t intake_count_{0};

  std::function<bool(std::size_t, const BallGamepieceSim&)> custom_condition_{};
  std::deque<ContactEvent> recent_events_{};
};

}  // namespace frcsim
