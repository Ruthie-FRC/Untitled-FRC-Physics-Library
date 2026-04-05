#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>
#include <vector>

#include "frcsim/math/vector.hpp"

namespace frcsim {

class ShotCalculator3D {
  public:
    struct TablePoint {
        double distance_m{0.0};
        double hood_pitch_rad{0.0};
        double muzzle_speed_mps{0.0};
        double time_of_flight_s{0.0};
    };

    struct Config {
        double min_distance_m{1.3};
        double max_distance_m{5.8};
        double phase_delay_s{0.03};
        double tof_scale{1.0};
        double valid_distance_epsilon{1e-6};

        double hood_offset_rad{0.0};
        double hood_distance_slope_radpm{0.0};
        double speed_offset_mps{0.0};
        double speed_distance_slope_mpspm{0.0};

        double recent_shot_history_window_s{10.0};
        double recent_shot_sample_period_s{0.1};
        double recent_pose_band_m{0.75};
        double recent_target_band_m{0.75};
    };

    struct ShotParameters {
        bool is_valid{false};
        double turret_yaw_rad{0.0};
        double hood_pitch_rad{0.0};
        double flywheel_speed_mps{0.0};
        double distance_m{0.0};
        double time_of_flight_s{0.0};
    };

    ShotCalculator3D() = default;

    explicit ShotCalculator3D(const Config& config) : config_(config) {}

    const Config& config() const { return config_; }
    void setConfig(const Config& config) { config_ = config; }

    void clearHistory() {
        recent_shot_samples_.clear();
        last_recent_sample_s_ = -std::numeric_limits<double>::infinity();
    }

    void setLookupTable(std::vector<TablePoint> points) {
        table_ = std::move(points);
        std::sort(table_.begin(), table_.end(), [](const TablePoint& a, const TablePoint& b) {
            return a.distance_m < b.distance_m;
        });
    }

    const std::vector<TablePoint>& lookupTable() const { return table_; }

    ShotParameters calculateShot(const Vector3& shooter_origin_m, const Vector3& robot_velocity_mps,
                                 const Vector3& target_position_m, double now_s) {
        ShotParameters result{};
        if (table_.empty()) {
            return result;
        }

        trimRecentShotHistory(now_s);

        const Vector3 delayed_origin = shooter_origin_m + robot_velocity_mps * config_.phase_delay_s;
        Vector3 lookahead_origin = delayed_origin;
        double lookahead_distance_m = planarDistance(lookahead_origin, target_position_m);

        for (int i = 0; i < 20; ++i) {
            const auto sampled = sampleTable(clampedDistance(lookahead_distance_m));
            const double tof_s = sampled.time_of_flight_s * std::max(0.0, config_.tof_scale);
            lookahead_origin = delayed_origin + robot_velocity_mps * tof_s;
            lookahead_distance_m = planarDistance(lookahead_origin, target_position_m);
        }

        const double distance_clamped = clampedDistance(lookahead_distance_m);
        const auto base = sampleTable(distance_clamped);

        const Vector3 target_delta = target_position_m - lookahead_origin;
        const double yaw_rad = std::atan2(target_delta.y, target_delta.x);

        double hood_pitch_rad = base.hood_pitch_rad;
        double flywheel_speed_mps = base.muzzle_speed_mps;

        const RecentShotSample* recent = findRecentShotSample(lookahead_origin, target_position_m);
        if (recent != nullptr) {
            const double pose_band = std::max(config_.valid_distance_epsilon, config_.recent_pose_band_m);
            const double pose_delta = (recent->robot_translation_m - lookahead_origin).norm();
            const double history_weight = 1.0 - std::min(1.0, pose_delta / pose_band);
            hood_pitch_rad = lerp(hood_pitch_rad, recent->hood_pitch_rad, history_weight);
            flywheel_speed_mps = lerp(flywheel_speed_mps, recent->flywheel_speed_mps, history_weight);
        }

        hood_pitch_rad += config_.hood_offset_rad;
        hood_pitch_rad += config_.hood_distance_slope_radpm * distance_clamped;

        flywheel_speed_mps += config_.speed_offset_mps;
        flywheel_speed_mps += config_.speed_distance_slope_mpspm * distance_clamped;

        const bool valid = lookahead_distance_m >= (config_.min_distance_m - config_.valid_distance_epsilon) &&
                           lookahead_distance_m <= (config_.max_distance_m + config_.valid_distance_epsilon);

        result.is_valid = valid;
        result.turret_yaw_rad = yaw_rad;
        result.hood_pitch_rad = hood_pitch_rad;
        result.flywheel_speed_mps = flywheel_speed_mps;
        result.distance_m = distance_clamped;
        result.time_of_flight_s = base.time_of_flight_s;

        if (valid) {
            saveRecentShotSample(now_s, lookahead_origin, target_position_m, distance_clamped, hood_pitch_rad,
                                 flywheel_speed_mps);
        }

        return result;
    }

  private:
    struct RecentShotSample {
        double timestamp_s{0.0};
        Vector3 robot_translation_m{};
        Vector3 target_position_m{};
        double distance_m{0.0};
        double hood_pitch_rad{0.0};
        double flywheel_speed_mps{0.0};
    };

    struct SampledValues {
        double hood_pitch_rad{0.0};
        double muzzle_speed_mps{0.0};
        double time_of_flight_s{0.0};
    };

    static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    double clampedDistance(double distance_m) const {
        return std::clamp(distance_m, config_.min_distance_m, config_.max_distance_m);
    }

    static double planarDistance(const Vector3& a, const Vector3& b) {
        const Vector3 delta = b - a;
        return std::sqrt(delta.x * delta.x + delta.y * delta.y);
    }

    SampledValues sampleTable(double distance_m) const {
        if (table_.empty()) {
            return {};
        }

        if (distance_m <= table_.front().distance_m) {
            return {table_.front().hood_pitch_rad, table_.front().muzzle_speed_mps, table_.front().time_of_flight_s};
        }
        if (distance_m >= table_.back().distance_m) {
            return {table_.back().hood_pitch_rad, table_.back().muzzle_speed_mps, table_.back().time_of_flight_s};
        }

        for (std::size_t i = 1; i < table_.size(); ++i) {
            const TablePoint& prev = table_[i - 1];
            const TablePoint& next = table_[i];
            if (distance_m > next.distance_m) {
                continue;
            }

            const double span = std::max(config_.valid_distance_epsilon, next.distance_m - prev.distance_m);
            const double t = std::clamp((distance_m - prev.distance_m) / span, 0.0, 1.0);
            return {
                lerp(prev.hood_pitch_rad, next.hood_pitch_rad, t),
                lerp(prev.muzzle_speed_mps, next.muzzle_speed_mps, t),
                lerp(prev.time_of_flight_s, next.time_of_flight_s, t),
            };
        }

        return {table_.back().hood_pitch_rad, table_.back().muzzle_speed_mps, table_.back().time_of_flight_s};
    }

    void trimRecentShotHistory(double now_s) {
        while (!recent_shot_samples_.empty() &&
               now_s - recent_shot_samples_.front().timestamp_s > config_.recent_shot_history_window_s) {
            recent_shot_samples_.pop_front();
        }
    }

    const RecentShotSample* findRecentShotSample(const Vector3& robot_translation_m,
                                                 const Vector3& target_position_m) const {
        const double pose_band = std::max(config_.valid_distance_epsilon, config_.recent_pose_band_m);
        const double target_band = std::max(config_.valid_distance_epsilon, config_.recent_target_band_m);

        const RecentShotSample* best = nullptr;
        double best_pose_delta = pose_band;

        for (const auto& sample : recent_shot_samples_) {
            const double target_delta = (sample.target_position_m - target_position_m).norm();
            if (target_delta > target_band) {
                continue;
            }

            const double pose_delta = (sample.robot_translation_m - robot_translation_m).norm();
            if (pose_delta <= best_pose_delta) {
                best_pose_delta = pose_delta;
                best = &sample;
            }
        }

        return best;
    }

    void saveRecentShotSample(double now_s, const Vector3& robot_translation_m, const Vector3& target_position_m,
                              double distance_m, double hood_pitch_rad, double flywheel_speed_mps) {
        if (now_s - last_recent_sample_s_ < config_.recent_shot_sample_period_s) {
            return;
        }

        recent_shot_samples_.push_back(
            {now_s, robot_translation_m, target_position_m, distance_m, hood_pitch_rad, flywheel_speed_mps});
        last_recent_sample_s_ = now_s;
        trimRecentShotHistory(now_s);
    }

    Config config_{};
    std::vector<TablePoint> table_{};

    std::deque<RecentShotSample> recent_shot_samples_{};
    double last_recent_sample_s_{-std::numeric_limits<double>::infinity()};
};

}  // namespace frcsim
