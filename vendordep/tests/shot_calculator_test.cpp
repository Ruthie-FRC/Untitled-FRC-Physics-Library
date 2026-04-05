#include <cassert>
#include <cmath>
#include <limits>
#include <vector>

#include "frcsim/mechanisms/shot_calculator.hpp"
#include "frcsim/mechanisms/turret_shooter_sim.hpp"

int main() {
    frcsim::ShotCalculator3D::Config config;
    config.min_distance_m = 1.0;
    config.max_distance_m = 6.0;
    config.phase_delay_s = 0.03;
    config.tof_scale = 1.0;
    config.recent_pose_band_m = 0.5;
    config.ballistic_refinement_enabled = false;

    frcsim::ShotCalculator3D calculator(config);
    calculator.setLookupTable(std::vector<frcsim::ShotCalculator3D::TablePoint>{
        {1.0, 0.65, 12.0, 0.22},
        {3.0, 0.80, 16.0, 0.36},
        {6.0, 1.02, 22.0, 0.58},
    });

    const frcsim::Vector3 target(5.0, 0.0, 2.1);
    const frcsim::Vector3 shooter_origin(0.0, 0.0, 1.0);
    const frcsim::Vector3 robot_velocity(1.0, 0.4, 0.0);

    auto shot = calculator.calculateShot(shooter_origin, robot_velocity, target, 1.0);
    assert(shot.is_valid);
    assert(shot.flywheel_speed_mps > 15.0 && shot.flywheel_speed_mps < 22.5);
    assert(shot.hood_pitch_rad > 0.7 && shot.hood_pitch_rad < 1.05);

    // With lateral velocity, yaw should bias opposite the lookahead direction enough to differ from zero.
    assert(std::fabs(shot.turret_yaw_rad) > 1e-3);

    // Recent-shot blending path: second query near first pose should stay valid and finite.
    auto shot_blended = calculator.calculateShot(
        shooter_origin + frcsim::Vector3(0.04, 0.01, 0.0), robot_velocity, target, 1.15);
    assert(shot_blended.is_valid);
    assert(std::isfinite(shot_blended.hood_pitch_rad));
    assert(std::isfinite(shot_blended.flywheel_speed_mps));

    // Refinement should provide a drag-aware correction on top of table interpolation.
    frcsim::ShotCalculator3D::Config draggy = config;
    draggy.min_distance_m = 1.0;
    draggy.max_distance_m = 8.0;
    draggy.ballistic_refinement_enabled = false;
    draggy.air_density_kgpm3 = 1.225;
    draggy.drag_scale = 1.0;
    draggy.projectile_mass_kg = 0.27;
    draggy.projectile_drag_coefficient = 0.47;
    draggy.projectile_reference_area_m2 = 0.025;
    draggy.assumed_spin_radps = frcsim::Vector3::zero();

    frcsim::ShotCalculator3D baseline_calc(draggy);
    baseline_calc.setLookupTable(std::vector<frcsim::ShotCalculator3D::TablePoint>{
        {1.0, 0.40, 8.0, 0.20},
        {4.0, 0.46, 10.0, 0.45},
        {8.0, 0.52, 12.0, 0.90},
    });

    frcsim::ShotCalculator3D::Config refined_cfg = draggy;
    refined_cfg.ballistic_refinement_enabled = true;
    refined_cfg.ballistic_refinement_iterations = 5;
    refined_cfg.ballistic_pitch_gain_radpm = 0.05;
    refined_cfg.ballistic_speed_gain_mpspm = 0.9;
    frcsim::ShotCalculator3D refined_calc(refined_cfg);
    refined_calc.setLookupTable(std::vector<frcsim::ShotCalculator3D::TablePoint>{
        {1.0, 0.40, 8.0, 0.20},
        {4.0, 0.46, 10.0, 0.45},
        {8.0, 0.52, 12.0, 0.90},
    });

    const frcsim::Vector3 far_target(7.0, 0.0, 2.2);
    const auto baseline_shot = baseline_calc.calculateShot(shooter_origin, frcsim::Vector3::zero(), far_target, 2.0);
    const auto refined_shot = refined_calc.calculateShot(shooter_origin, frcsim::Vector3::zero(), far_target, 2.0);
    assert(baseline_shot.is_valid);
    assert(refined_shot.is_valid);
    assert(std::isfinite(refined_shot.hood_pitch_rad));
    assert(std::isfinite(refined_shot.flywheel_speed_mps));
    assert(refined_shot.flywheel_speed_mps >= baseline_shot.flywheel_speed_mps);

    // Invalid numeric inputs should fail closed.
    const auto bad_query = calculator.calculateShot(
        frcsim::Vector3(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0),
        frcsim::Vector3::zero(),
        target,
        2.5);
    assert(!bad_query.is_valid);

    // Integration with turret differential math.
    frcsim::TurretShooterSim sim;
    const bool applied = sim.applyAim(shot.turret_yaw_rad, shot.hood_pitch_rad);
    assert(applied);

    const auto solved = sim.jointState();
    assert(std::fabs(solved.yaw_rad - shot.turret_yaw_rad) < 1e-6);
    assert(std::fabs(solved.pitch_rad - shot.hood_pitch_rad) < 1e-6);

    return 0;
}
