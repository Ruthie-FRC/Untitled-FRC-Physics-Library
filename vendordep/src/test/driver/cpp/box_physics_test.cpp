#include "gtest/gtest.h"

#include "frcsim/gamepiece/box_gamepiece_presets.hpp"
#include "frcsim/gamepiece/box_physics.hpp"

namespace {

TEST(BoxPhysicsSimTest, StoresConfiguredDimensions) {
  const auto props = frcsim::BoxGamepiecePresets::season2026BoxProperties();
  const frcsim::BoxPhysicsSim3D sim(
      frcsim::BoxGamepiecePresets::season2026BoxConfig(), props);

  EXPECT_DOUBLE_EQ(sim.boxProperties().dimensions_m.x, 0.30);
  EXPECT_DOUBLE_EQ(sim.boxProperties().dimensions_m.y, 0.30);
  EXPECT_DOUBLE_EQ(sim.boxProperties().dimensions_m.z, 0.20);
}

TEST(BoxPhysicsSimTest, GroundContactUsesHalfHeightFromCenter) {
  frcsim::BoxPhysicsSim3D::Config config =
      frcsim::BoxGamepiecePresets::evergreenBoxConfig();
  config.gravity_mps2 = frcsim::Vector3(0.0, 0.0, 0.0);
  config.rolling_friction_per_s = 0.0;

  frcsim::BoxPhysicsSim3D::BoxProperties short_props{};
  short_props.dimensions_m = frcsim::Vector3(0.50, 0.50, 0.20);

  frcsim::BoxPhysicsSim3D::BoxProperties tall_props{};
  tall_props.dimensions_m = frcsim::Vector3(0.50, 0.50, 0.60);

  frcsim::BoxPhysicsSim3D short_box(config, short_props);
  frcsim::BoxPhysicsSim3D tall_box(config, tall_props);

  frcsim::BoxPhysicsSim3D::BoxState short_state;
  short_state.position_m = frcsim::Vector3(0.0, 0.0, 0.0);
  short_box.setState(short_state);

  frcsim::BoxPhysicsSim3D::BoxState tall_state;
  tall_state.position_m = frcsim::Vector3(0.0, 0.0, 0.0);
  tall_box.setState(tall_state);

  short_box.step(0.01);
  tall_box.step(0.01);

  EXPECT_NEAR(short_box.state().position_m.z, 0.10, 1e-9);
  EXPECT_NEAR(tall_box.state().position_m.z, 0.30, 1e-9);
}

}  // namespace