#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>
#include <vector>

#include "frcsim/gamepiece/ball_physics.hpp"
#include "frcsim/math/vector.hpp"

namespace frcsim {

/**
 * @brief 3D shot solution generator using table interpolation plus optional ballistic refinement.
 */
class ShotCalculator3D {
  public:
        /** @brief Lookup-table row for distance-indexed shot tuning. */
    struct TablePoint {
        double distance_m{0.0};
        double hood_pitch_rad{0.0};
        double muzzle_speed_mps{0.0};
        double time_of_flight_s{0.0};
    };

    /** @brief Solver configuration for interpolation, refinement, and historical smoothing behavior. */
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

        // Ballistic correction pass layered on top of table interpolation.
        bool ballistic_refinement_enabled{true};
        int ballistic_refinement_iterations{3};
        double ballistic_time_step_s{0.01};
        double ballistic_max_time_s{2.5};
        double ballistic_pitch_gain_radpm{0.035};
        double ballistic_speed_gain_mpspm{0.75};

        Vector3 gravity_mps2{0.0, 0.0, -9.81};
        double effective_gravity_scale{1.0};
        double air_density_kgpm3{1.225};
        double drag_scale{1.0};
        double magnus_coefficient{1e-4};
        double magnus_scale{1.0};

        double projectile_mass_kg{0.27};
        double projectile_drag_coefficient{0.47};
        double projectile_reference_area_m2{0.025};
        Vector3 assumed_spin_radps{};

        double min_pitch_rad{-0.2};
        double max_pitch_rad{1.35};
        double min_speed_mps{0.0};
        double max_speed_mps{45.0};
    };

    /** @brief Output shot command/result packet. */
    struct ShotParameters {
        bool is_valid{false};
        double turret_yaw_rad{0.0};
        double hood_pitch_rad{0.0};
        double flywheel_speed_mps{0.0};
        double distance_m{0.0};
        double time_of_flight_s{0.0};
    };

    ShotCalculator3D() = default;

    /**
     * @brief Constructs calculator with explicit configuration.
     * @param config Initial calculator configuration.
     */
    explicit ShotCalculator3D(const Config& config) : config_(sanitizeConfig(config)) {}

    /** @brief Returns active sanitized configuration. @return Immutable config reference. */
    const Config& config() const { return config_; }
    /** @brief Replaces configuration after sanitization. @param config New configuration. */
    void setConfig(const Config& config) { config_ = sanitizeConfig(config); }

    /** @brief Clears learned recent-shot history used for contextual blending. */
    void clearHistory() {
        recent_shot_samples_.clear();
        last_recent_sample_s_ = -std::numeric_limits<double>::infinity();
    }

    /**
     * @brief Replaces lookup table with validated and distance-sorted points.
     * @param points Candidate table points to ingest.
     */
    void setLookupTable(std::vector<TablePoint> points) {
        table_.clear();
        table_.reserve(points.size());
        for (const auto& point : points) {
            if (!std::isfinite(point.distance_m) || !std::isfinite(point.hood_pitch_rad) ||
                !std::isfinite(point.muzzle_speed_mps) || !std::isfinite(point.time_of_flight_s)) {
                continue;
            }
            table_.push_back({
                std::max(0.0, point.distance_m),
                point.hood_pitch_rad,
                std::max(0.0, point.muzzle_speed_mps),
                std::max(0.0, point.time_of_flight_s),
            });
        }
        std::sort(table_.begin(), table_.end(), [](const TablePoint& a, const TablePoint& b) {
            return a.distance_m < b.distance_m;
        });
    }

    /** @brief Returns current lookup table. @return Immutable vector reference of table points. */
    const std::vector<TablePoint>& lookupTable() const { return table_; }

    /**
     * @brief Computes a shot solution for current robot/target kinematics.
     * @param shooter_origin_m Shooter origin in world coordinates.
     * @param robot_velocity_mps Robot chassis velocity in world frame.
     * @param target_position_m Target center position in world coordinates.
     * @param now_s Current timestamp in seconds.
     * @return ShotParameters with validity flag and computed yaw/pitch/speed/time.
     */
    ShotParameters calculateShot(const Vector3& shooter_origin_m, const Vector3& robot_velocity_mps,
                                 const Vector3& target_position_m, double now_s) {
        ShotParameters result{};
        if (table_.empty() || !finiteVector(shooter_origin_m) || !finiteVector(robot_velocity_mps) ||
            !finiteVector(target_position_m) || !std::isfinite(now_s)) {
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
        const double effective_tof_s = base.time_of_flight_s * std::max(0.0, config_.tof_scale);

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

        hood_pitch_rad = std::clamp(hood_pitch_rad, config_.min_pitch_rad, config_.max_pitch_rad);
        flywheel_speed_mps = std::clamp(flywheel_speed_mps, config_.min_speed_mps, config_.max_speed_mps);

        if (config_.ballistic_refinement_enabled) {
            refineBallisticShot(lookahead_origin, target_position_m, yaw_rad, hood_pitch_rad, flywheel_speed_mps);
        }

        const bool valid = lookahead_distance_m >= (config_.min_distance_m - config_.valid_distance_epsilon) &&
                           lookahead_distance_m <= (config_.max_distance_m + config_.valid_distance_epsilon);

        result.is_valid = valid;
        result.turret_yaw_rad = yaw_rad;
        result.hood_pitch_rad = hood_pitch_rad;
        result.flywheel_speed_mps = flywheel_speed_mps;
        result.distance_m = distance_clamped;
        result.time_of_flight_s = effective_tof_s;

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

    static bool finiteVector(const Vector3& v) {
        return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
    }

    static double sanitizeNonNegative(double value, double fallback) {
        if (!std::isfinite(value)) {
            return fallback;
        }
        return std::max(0.0, value);
    }

    static Config sanitizeConfig(const Config& config) {
        Config sanitized = config;

        sanitized.valid_distance_epsilon = std::clamp(sanitizeNonNegative(sanitized.valid_distance_epsilon, 1e-6), 1e-9, 1e-2);
        sanitized.min_distance_m = sanitizeNonNegative(sanitized.min_distance_m, 1.3);
        sanitized.max_distance_m = sanitizeNonNegative(sanitized.max_distance_m, 5.8);
        if (sanitized.max_distance_m < sanitized.min_distance_m + sanitized.valid_distance_epsilon) {
            sanitized.max_distance_m = sanitized.min_distance_m + 1.0;
        }

        sanitized.phase_delay_s = std::isfinite(sanitized.phase_delay_s) ? sanitized.phase_delay_s : 0.03;
        sanitized.tof_scale = sanitizeNonNegative(sanitized.tof_scale, 1.0);

        sanitized.recent_shot_history_window_s = sanitizeNonNegative(sanitized.recent_shot_history_window_s, 10.0);
        sanitized.recent_shot_sample_period_s = sanitizeNonNegative(sanitized.recent_shot_sample_period_s, 0.1);
        sanitized.recent_pose_band_m = std::max(sanitized.valid_distance_epsilon,
                                                sanitizeNonNegative(sanitized.recent_pose_band_m, 0.75));
        sanitized.recent_target_band_m = std::max(sanitized.valid_distance_epsilon,
                                                  sanitizeNonNegative(sanitized.recent_target_band_m, 0.75));

        sanitized.ballistic_refinement_iterations = std::clamp(sanitized.ballistic_refinement_iterations, 0, 10);
        sanitized.ballistic_time_step_s = std::clamp(sanitizeNonNegative(sanitized.ballistic_time_step_s, 0.01), 1e-4, 0.05);
        sanitized.ballistic_max_time_s = std::clamp(sanitizeNonNegative(sanitized.ballistic_max_time_s, 2.5), 0.1, 10.0);
        sanitized.ballistic_pitch_gain_radpm = std::clamp(
            std::isfinite(sanitized.ballistic_pitch_gain_radpm) ? sanitized.ballistic_pitch_gain_radpm : 0.035,
            0.0, 0.4);
        sanitized.ballistic_speed_gain_mpspm = std::clamp(
            std::isfinite(sanitized.ballistic_speed_gain_mpspm) ? sanitized.ballistic_speed_gain_mpspm : 0.75,
            0.0, 10.0);

        if (!finiteVector(sanitized.gravity_mps2)) {
            sanitized.gravity_mps2 = Vector3(0.0, 0.0, -9.81);
        }
        sanitized.effective_gravity_scale = std::clamp(sanitizeNonNegative(sanitized.effective_gravity_scale, 1.0), 0.0, 5.0);
        sanitized.air_density_kgpm3 = std::clamp(sanitizeNonNegative(sanitized.air_density_kgpm3, 1.225), 0.0, 5.0);
        sanitized.drag_scale = std::clamp(sanitizeNonNegative(sanitized.drag_scale, 1.0), 0.0, 10.0);
        sanitized.magnus_coefficient = std::isfinite(sanitized.magnus_coefficient) ? sanitized.magnus_coefficient : 1e-4;
        sanitized.magnus_scale = std::clamp(sanitizeNonNegative(sanitized.magnus_scale, 1.0), 0.0, 10.0);

        sanitized.projectile_mass_kg = std::max(1e-6, sanitizeNonNegative(sanitized.projectile_mass_kg, 0.27));
        sanitized.projectile_drag_coefficient = std::clamp(
            sanitizeNonNegative(sanitized.projectile_drag_coefficient, 0.47), 0.0, 5.0);
        sanitized.projectile_reference_area_m2 = sanitizeNonNegative(sanitized.projectile_reference_area_m2, 0.025);
        if (sanitized.projectile_reference_area_m2 <= 0.0) {
            sanitized.projectile_reference_area_m2 = 0.025;
        }
        if (!finiteVector(sanitized.assumed_spin_radps)) {
            sanitized.assumed_spin_radps = Vector3::zero();
        }

        sanitized.min_pitch_rad = std::isfinite(sanitized.min_pitch_rad) ? sanitized.min_pitch_rad : -0.2;
        sanitized.max_pitch_rad = std::isfinite(sanitized.max_pitch_rad) ? sanitized.max_pitch_rad : 1.35;
        if (sanitized.max_pitch_rad < sanitized.min_pitch_rad) {
            std::swap(sanitized.min_pitch_rad, sanitized.max_pitch_rad);
        }
        sanitized.min_speed_mps = sanitizeNonNegative(sanitized.min_speed_mps, 0.0);
        sanitized.max_speed_mps = sanitizeNonNegative(sanitized.max_speed_mps, 45.0);
        if (sanitized.max_speed_mps < sanitized.min_speed_mps) {
            sanitized.max_speed_mps = sanitized.min_speed_mps + 1.0;
        }

        return sanitized;
    }

    double clampedDistance(double distance_m) const {
        return std::clamp(distance_m, config_.min_distance_m, config_.max_distance_m);
    }

    static double planarDistance(const Vector3& a, const Vector3& b) {
        const Vector3 delta = b - a;
        return std::sqrt(delta.x * delta.x + delta.y * delta.y);
    }

    struct BallisticSample {
        bool reached_target_range{false};
        Vector3 position_m{};
        double travelled_range_m{0.0};
        double time_s{0.0};
    };

    Vector3 ballisticAcceleration(const Vector3& velocity_mps) const {
        const Vector3 gravity_mps2 = config_.gravity_mps2 * config_.effective_gravity_scale;
        const Vector3 drag_force_n =
            Vector3::dragForce(velocity_mps, config_.projectile_drag_coefficient,
                               config_.projectile_reference_area_m2, config_.air_density_kgpm3) * config_.drag_scale;
        const Vector3 magnus_force_n =
            Vector3::magnusForce(velocity_mps, config_.assumed_spin_radps, config_.magnus_coefficient) *
            config_.magnus_scale;
        const Vector3 accel_mps2 =
            gravity_mps2 + (drag_force_n + magnus_force_n) * (1.0 / std::max(1e-9, config_.projectile_mass_kg));
        return finiteVector(accel_mps2) ? accel_mps2 : gravity_mps2;
    }

    BallisticSample simulateAtRange(const Vector3& origin_m, double yaw_rad, double pitch_rad, double speed_mps,
                                    double target_planar_range_m) const {
        BallisticSample sample{};
        sample.position_m = origin_m;

        const double cos_pitch = std::cos(pitch_rad);
        Vector3 velocity_mps(
            speed_mps * cos_pitch * std::cos(yaw_rad),
            speed_mps * cos_pitch * std::sin(yaw_rad),
            speed_mps * std::sin(pitch_rad));

        const Vector3 forward(std::cos(yaw_rad), std::sin(yaw_rad), 0.0);
        const double dt_s = config_.ballistic_time_step_s;

        Vector3 previous_position_m = sample.position_m;
        double previous_range_m = 0.0;

        while (sample.time_s < config_.ballistic_max_time_s) {
            previous_position_m = sample.position_m;
            previous_range_m = sample.travelled_range_m;

            const Vector3 accel0_mps2 = ballisticAcceleration(velocity_mps);
            const Vector3 mid_velocity_mps = velocity_mps + accel0_mps2 * (0.5 * dt_s);
            const Vector3 accel_mid_mps2 = ballisticAcceleration(mid_velocity_mps);

            velocity_mps += accel_mid_mps2 * dt_s;
            sample.position_m += mid_velocity_mps * dt_s;
            sample.time_s += dt_s;

            const Vector3 offset_m = sample.position_m - origin_m;
            sample.travelled_range_m = offset_m.x * forward.x + offset_m.y * forward.y;

            if (!finiteVector(sample.position_m) || !std::isfinite(sample.travelled_range_m)) {
                sample.position_m = previous_position_m;
                sample.travelled_range_m = previous_range_m;
                break;
            }

            if (sample.travelled_range_m >= target_planar_range_m) {
                const double denom = std::max(config_.valid_distance_epsilon,
                                              sample.travelled_range_m - previous_range_m);
                const double alpha = std::clamp((target_planar_range_m - previous_range_m) / denom, 0.0, 1.0);
                sample.position_m = previous_position_m + (sample.position_m - previous_position_m) * alpha;
                sample.travelled_range_m = target_planar_range_m;
                sample.reached_target_range = true;
                break;
            }
        }

        return sample;
    }

    void refineBallisticShot(const Vector3& origin_m, const Vector3& target_position_m, double yaw_rad,
                             double& hood_pitch_rad, double& flywheel_speed_mps) const {
        const double target_planar_range_m = planarDistance(origin_m, target_position_m);
        if (!std::isfinite(target_planar_range_m) || target_planar_range_m < config_.valid_distance_epsilon) {
            return;
        }

        for (int i = 0; i < config_.ballistic_refinement_iterations; ++i) {
            const auto sample =
                simulateAtRange(origin_m, yaw_rad, hood_pitch_rad, flywheel_speed_mps, target_planar_range_m);

            if (!sample.reached_target_range) {
                const double shortfall_m = std::max(0.0, target_planar_range_m - sample.travelled_range_m);
                flywheel_speed_mps += std::clamp(shortfall_m * config_.ballistic_speed_gain_mpspm, 0.0, 2.0);
                flywheel_speed_mps = std::clamp(flywheel_speed_mps, config_.min_speed_mps, config_.max_speed_mps);
                continue;
            }

            const double vertical_error_m = target_position_m.z - sample.position_m.z;
            hood_pitch_rad += std::clamp(vertical_error_m * config_.ballistic_pitch_gain_radpm, -0.08, 0.08);
            flywheel_speed_mps +=
                std::clamp(vertical_error_m * config_.ballistic_speed_gain_mpspm * 0.5, -0.6, 0.6);

            hood_pitch_rad = std::clamp(hood_pitch_rad, config_.min_pitch_rad, config_.max_pitch_rad);
            flywheel_speed_mps = std::clamp(flywheel_speed_mps, config_.min_speed_mps, config_.max_speed_mps);
        }
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
