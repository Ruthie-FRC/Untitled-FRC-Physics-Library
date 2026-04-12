#include <cmath>

#include "driverheader.h"
#include "frcsim/field/boundary.hpp"
#include "frcsim/physics_world.hpp"
#include "gtest/gtest.h"

namespace {

constexpr double kEps = 1e-6;

TEST(PhysicsWorldContactTest, PlaneContactAppliesBounceAndFriction) {
  frcsim::PhysicsConfig config;
  config.fixed_dt_s = 0.01;
  config.enable_gravity = false;
  config.enable_collision_detection = true;

  frcsim::PhysicsWorld world(config);
  frcsim::RigidBody& body = world.createBody(1.0);
  body.setPosition(0.0, 0.0, -0.02);
  body.setLinearVelocity(2.0, 0.0, -1.0);

  frcsim::Material mat;
  mat.coefficient_of_restitution = 0.6;
  mat.coefficient_of_friction_kinetic = 0.8;
  body.setMaterial(mat);

  auto& ground = world.addBoundary();
  ground.type = frcsim::BoundaryType::kPlane;
  ground.position_m = frcsim::Vector3(0.0, 0.0, 0.0);
  ground.restitution = 0.4;
  ground.friction_coefficient = 0.9;

  world.step();

  EXPECT_GE(body.position().z, -kEps);
  EXPECT_GT(body.linearVelocity().z, 0.0);
  EXPECT_LT(body.linearVelocity().x, 2.0);
}

TEST(PhysicsWorldContactTest, CollisionFilterMaskBlocksBoundaryResponse) {
  frcsim::PhysicsConfig config;
  config.fixed_dt_s = 0.01;
  config.enable_gravity = false;
  config.enable_collision_detection = true;

  frcsim::PhysicsWorld world(config);
  frcsim::RigidBody& body = world.createBody(1.0);
  body.setPosition(0.0, 0.0, -0.02);
  body.setLinearVelocity(0.0, 0.0, -1.0);
  body.setCollisionLayer(0x2u);
  body.setCollisionMask(0x2u);

  auto& ground = world.addBoundary();
  ground.type = frcsim::BoundaryType::kPlane;
  ground.position_m = frcsim::Vector3(0.0, 0.0, 0.0);
  ground.collision_layer_bits = 0x1u;
  ground.collision_mask_bits = 0x1u;

  world.step();

  // No interaction because masks do not overlap.
  EXPECT_LT(body.position().z, -kEps);
  EXPECT_LT(body.linearVelocity().z, 0.0);
}

TEST(PhysicsWorldContactTest, MaterialInteractionTableOverridesCoefficients) {
  frcsim::PhysicsConfig config;
  config.fixed_dt_s = 0.01;
  config.enable_gravity = false;
  config.enable_collision_detection = true;

  frcsim::PhysicsWorld world(config);
  frcsim::RigidBody& body = world.createBody(1.0);
  body.setPosition(0.0, 0.0, -0.02);
  body.setLinearVelocity(1.0, 0.0, -1.0);
  body.setMaterialId(10);

  auto& ground = world.addBoundary();
  ground.type = frcsim::BoundaryType::kPlane;
  ground.position_m = frcsim::Vector3(0.0, 0.0, 0.0);
  ground.material_id = 20;

  frcsim::PhysicsWorld::MaterialInteraction pair;
  pair.material_a_id = 10;
  pair.material_b_id = 20;
  pair.restitution = 1.0;
  pair.friction = 0.0;
  pair.enabled = true;
  world.setMaterialInteraction(pair);

  world.step();

  // Full bounce and no tangential damping from override.
  EXPECT_NEAR(body.linearVelocity().z, 1.0, 5e-2);
  EXPECT_NEAR(body.linearVelocity().x, 1.0, 5e-2);
}

TEST(DriverApiExportTest, PoseAndVelocityArraysMatchBodyState) {
  const uint64_t world = c_rsCreateWorld(0.01, 0);
  ASSERT_NE(world, 0u);

  const int body = c_rsCreateBody(world, 5.0);
  ASSERT_GE(body, 0);

  ASSERT_EQ(c_rsSetBodyPosition(world, body, 1.0, 2.0, 3.0), 0);
  ASSERT_EQ(c_rsSetBodyLinearVelocity(world, body, 4.0, 5.0, 6.0), 0);

  double pose7[7] = {};
  const int pose_count = c_rsGetBodyPose7Array(world, pose7, 1);
  ASSERT_EQ(pose_count, 1);
  EXPECT_NEAR(pose7[0], 1.0, kEps);
  EXPECT_NEAR(pose7[1], 2.0, kEps);
  EXPECT_NEAR(pose7[2], 3.0, kEps);
  EXPECT_NEAR(pose7[3], 1.0, kEps);
  EXPECT_NEAR(pose7[4], 0.0, kEps);
  EXPECT_NEAR(pose7[5], 0.0, kEps);
  EXPECT_NEAR(pose7[6], 0.0, kEps);

  double velocity6[6] = {};
  const int vel_count = c_rsGetBodyVelocity6Array(world, velocity6, 1);
  ASSERT_EQ(vel_count, 1);
  EXPECT_NEAR(velocity6[0], 4.0, kEps);
  EXPECT_NEAR(velocity6[1], 5.0, kEps);
  EXPECT_NEAR(velocity6[2], 6.0, kEps);
  EXPECT_NEAR(velocity6[3], 0.0, kEps);
  EXPECT_NEAR(velocity6[4], 0.0, kEps);
  EXPECT_NEAR(velocity6[5], 0.0, kEps);

  c_rsDestroyWorld(world);
}

TEST(DriverApiAeroTest, ConfigurableAerodynamicsSlowsBody) {
  const uint64_t world = c_rsCreateWorld(0.01, 0);
  ASSERT_NE(world, 0u);

  const int body = c_rsCreateBody(world, 1.0);
  ASSERT_GE(body, 0);

  ASSERT_EQ(c_rsSetBodyLinearVelocity(world, body, 10.0, 0.0, 0.0), 0);
  ASSERT_EQ(c_rsSetBodyAerodynamicSphere(world, body, 0.12, 0.47), 0);
  ASSERT_EQ(c_rsSetWorldAerodynamics(world, 1, 1.225, 0.2, 1e-4, 0.47, 0.01), 0);

  ASSERT_EQ(c_rsStepWorld(world, 50), 0);

  double out_v[3] = {};
  ASSERT_EQ(c_rsGetBodyLinearVelocity(world, body, &out_v[0], &out_v[1], &out_v[2]), 0);
  EXPECT_LT(out_v[0], 10.0);

  c_rsDestroyWorld(world);
}

TEST(DriverApiFilterMaterialTest, BodyFilterAndMaterialApisReturnSuccess) {
  const uint64_t world = c_rsCreateWorld(0.01, 0);
  ASSERT_NE(world, 0u);

  const int body = c_rsCreateBody(world, 1.0);
  ASSERT_GE(body, 0);

  ASSERT_EQ(c_rsSetBodyMaterialId(world, body, 7), 0);
  ASSERT_EQ(c_rsSetBodyCollisionFilter(world, body, 0x4u, 0x8u), 0);
  ASSERT_EQ(c_rsSetMaterialInteraction(world, 7, 9, 0.2, 0.9, 1), 0);

  c_rsDestroyWorld(world);
}

}  // namespace
