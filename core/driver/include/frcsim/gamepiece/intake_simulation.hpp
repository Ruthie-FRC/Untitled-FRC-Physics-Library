#pragma once

#include <cstddef>
#include <deque>
#include <functional>
#include <string>

#include "frcsim/gamepiece/ball_gamepiece_sim.hpp"

namespace frcsim {

class IntakeSimulation {
  public:
    struct Config {
        std::size_t robot_index{0};
        std::string targeted_type{"Ball"};
        std::size_t capacity{1};
    };

    struct ContactEvent {
        enum class Phase {
            kBegin,
            kPersist,
            kEnd,
        };

        std::size_t ball_index{BallGamepieceSim::kNoBall};
        Phase phase{Phase::kBegin};
    };

    explicit IntakeSimulation(const Config& config = Config()) : config_(config) {}

    void setConfig(const Config& config) { config_ = config; }
    const Config& config() const { return config_; }

    void setRunning(bool running) { running_ = running; }
    bool isRunning() const { return running_; }

    void setCustomIntakeCondition(const std::function<bool(std::size_t, const BallGamepieceSim&)>& predicate) {
        custom_condition_ = predicate;
    }

    std::size_t gamePiecesInIntakeCount() const { return intake_count_; }

    bool obtainGamePieceFromIntake() {
        if (intake_count_ == 0) {
            return false;
        }
        --intake_count_;
        return true;
    }

    const std::deque<ContactEvent>& recentEvents() const { return recent_events_; }

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
            if (!config_.targeted_type.empty() && sim.ballType(i) != config_.targeted_type) {
                continue;
            }

            const double distance = (ball.sim.state().position_m - intake_world).norm();
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
