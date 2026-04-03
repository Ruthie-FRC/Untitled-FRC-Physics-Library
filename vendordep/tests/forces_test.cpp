#include <cassert>
#include <cmath>
#include <iostream>

#include "frcsim/physics_world.hpp"
#include "frcsim/forces/gravity.hpp"

int main() {
    std::cout << "Testing forces and aerodynamics...\n";

    // ===== Basic Applied Force Tests =====
    {
        frcsim::PhysicsConfig config;
        config.fixed_dt_s = 0.01;
        config.enable_collision_detection = false;
        config.enable_joint_constraints = false;
        config.linear_damping_per_s = 0.0;
        config.angular_damping_per_s = 0.0;
        config.enable_gravity = false;

        frcsim::RigidBody body(1.0);

        body.applyForce(frcsim::Vector3(10.0, 0.0, 0.0));
        body.integrate(
            config.fixed_dt_s,
            config.integration_method,
            frcsim::Vector3::zero(),
            config.linear_damping_per_s,
            config.angular_damping_per_s);

        assert(std::fabs(body.linearVelocity().x - 0.1) < 1e-9);
        std::cout << "  ✓ Direct force application works\n";
    }

    // ===== Gravity Force via Global Config =====
    {
        frcsim::PhysicsConfig config;
        config.fixed_dt_s = 0.01;
        config.enable_collision_detection = false;
        config.enable_joint_constraints = false;
        config.linear_damping_per_s = 0.0;
        config.angular_damping_per_s = 0.0;
        config.enable_gravity = true;
        config.gravity_mps2 = frcsim::Vector3(0.0, 0.0, -9.81);

        frcsim::PhysicsWorld world(config);
        frcsim::RigidBody& body = world.createBody(1.0);
        body.setPosition(frcsim::Vector3(0.0, 0.0, 10.0));

        for (int i = 0; i < 100; ++i) {
            world.step();
        }

        assert(body.position().z < 6.0);
        assert(body.linearVelocity().z < -9.0);
        std::cout << "  ✓ Global gravity force works\n";
    }

    // ===== Gravity Force Generator Class =====
    {
        frcsim::PhysicsConfig config;
        config.fixed_dt_s = 0.01;
        config.enable_collision_detection = false;
        config.enable_joint_constraints = false;
        config.linear_damping_per_s = 0.0;
        config.angular_damping_per_s = 0.0;
        config.enable_gravity = false;

        frcsim::PhysicsWorld world(config);
        frcsim::RigidBody& body = world.createBody(1.0);
        body.setPosition(frcsim::Vector3(0.0, 0.0, 10.0));

        auto gravity = std::make_shared<frcsim::GravityForce>(
            frcsim::Vector3(0.0, 0.0, -9.81));
        world.addGlobalForceGenerator(gravity);

        for (int i = 0; i < 100; ++i) {
            world.step();
        }

        assert(body.position().z < 6.0);
        assert(body.linearVelocity().z < -9.0);
        std::cout << "  ✓ Gravity force generator works\n";
    }

    // ===== Multiple Force Accumulation =====
    {
        frcsim::PhysicsConfig config;
        config.fixed_dt_s = 0.01;
        config.enable_collision_detection = false;
        config.enable_joint_constraints = false;
        config.linear_damping_per_s = 0.0;
        config.angular_damping_per_s = 0.0;
        config.enable_gravity = false;

        frcsim::RigidBody body(1.0);

        body.applyForce(frcsim::Vector3(5.0, 0.0, 0.0));
        body.applyForce(frcsim::Vector3(5.0, 0.0, 0.0));
        
        body.integrate(
            config.fixed_dt_s,
            config.integration_method,
            frcsim::Vector3::zero(),
            config.linear_damping_per_s,
            config.angular_damping_per_s);

        assert(std::fabs(body.linearVelocity().x - 0.1) < 1e-6);
        std::cout << "  ✓ Force accumulation works\n";
    }

    // ===== Aerodynamics: Drag Model Tests =====
    {
        frcsim::PhysicsConfig config;
        config.fixed_dt_s = 0.01;
        config.enable_collision_detection = false;
        config.enable_joint_constraints = false;
        config.enable_gravity = false;
        config.enable_aerodynamics = true;
        config.linear_damping_per_s = 0.0;

        frcsim::PhysicsWorld world(config);
        frcsim::RigidBody& body = world.createBody(1.0);

        body.setLinearVelocity(frcsim::Vector3(10.0, 0.0, 0.0));

        for (int i = 0; i < 100; ++i) {
            world.step();
        }

        assert(body.linearVelocity().x < 10.0);
        std::cout << "  ✓ Drag model reduces velocity\n";
    }

    // ===== Aerodynamics: Detailed Drag Diagnostics =====
    {
        const auto details = frcsim::Vector3::dragForceDetailed(
            frcsim::Vector3(10.0, 0.0, 0.0), 0.47, 0.01);

        assert(details.valid);
        assert(std::fabs(details.speed_mps - 10.0) < 1e-9);
        assert(std::fabs(details.dynamic_pressure_pa - 61.25) < 1e-6);
        assert(details.force.x < 0.0);
        assert(std::fabs(details.force.y) < 1e-12);
        assert(std::fabs(details.force.z) < 1e-12);
        std::cout << "  ✓ Detailed drag diagnostics are populated\n";
    }

    // ===== Aerodynamics: Nonlinear Drag Includes Linear and Quadratic Terms =====
    {
        const auto details = frcsim::Vector3::dragForceDetailed(
            frcsim::Vector3(10.0, 0.0, 0.0), 0.47, 0.01, 1.225, 0.25);

        assert(details.valid);
        assert(details.linear_drag_coefficient_n_per_mps > 0.0);
        assert(details.quadratic_drag_coefficient_n_per_mps2 > 0.0);
        assert(details.drag_force_magnitude_n > 0.0);
        std::cout << "  ✓ Nonlinear drag terms are combined\n";
    }

    // ===== Aerodynamics: Drag Compared With Gravity =====
    {
        frcsim::DragModel drag_model(0.47, 0.01);
        frcsim::RigidBody body(2.0);
        body.setLinearVelocity(frcsim::Vector3(10.0, 0.0, 0.0));

        const auto comparison = drag_model.compareToEffectiveGravity(
            body, frcsim::Vector3(0.0, 0.0, -9.81));

        assert(comparison.valid);
        assert(comparison.drag_force_magnitude_n > 0.0);
        assert(comparison.effective_gravity_acceleration_mps2 > 0.0);
        assert(comparison.drag_to_gravity_ratio >= 0.0);
        std::cout << "  ✓ Drag-to-gravity comparison is available\n";
    }

    // ===== Aerodynamics: Body Geometry Controls Cross Section =====
    {
        frcsim::RigidBody body(1.0);
        frcsim::RigidBody::AerodynamicGeometry geometry;
        geometry.shape = frcsim::RigidBody::AerodynamicGeometry::Shape::kBox;
        geometry.box_dimensions_m = frcsim::Vector3(2.0, 1.0, 3.0);

        body.setAerodynamicGeometry(geometry);
        body.setOrientation(frcsim::Quaternion::fromAxisAngle(
            frcsim::Vector3::unitZ(), 1.5707963267948966));

        const double projected_area = body.dragReferenceAreaM2(frcsim::Vector3(1.0, 0.0, 0.0));

        assert(std::fabs(projected_area - 6.0) < 1e-9);

        frcsim::DragModel drag_model(0.47, 0.01);
        const auto details = drag_model.computeForceDetailed(body);

        assert(std::fabs(details.reference_area_m2 - 6.0) < 1e-9);
        std::cout << "  ✓ Body geometry drives projected drag area\n";
    }

    // ===== Aerodynamics: Cylinder Geometry Projects By Orientation =====
    {
        frcsim::RigidBody body(1.0);
        frcsim::RigidBody::AerodynamicGeometry geometry;
        geometry.shape = frcsim::RigidBody::AerodynamicGeometry::Shape::kCylinder;
        geometry.radius_m = 0.1;
        geometry.cylinder_length_m = 0.5;

        body.setAerodynamicGeometry(geometry);
        body.setOrientation(frcsim::Quaternion());

        const double area_end_on = body.dragReferenceAreaM2(frcsim::Vector3(0.0, 0.0, 10.0));
        const double area_side_on = body.dragReferenceAreaM2(frcsim::Vector3(10.0, 0.0, 0.0));

        const double expected_end_on = 3.14159265358979323846 * 0.1 * 0.1;
        const double expected_side_on = 2.0 * 0.1 * 0.5;

        assert(std::fabs(area_end_on - expected_end_on) < 1e-9);
        assert(std::fabs(area_side_on - expected_side_on) < 1e-9);
        std::cout << "  ✓ Cylinder projected drag area follows orientation\n";
    }

    // ===== Aerodynamics: Cylinder Axis Convenience Setter =====
    {
        frcsim::RigidBody body(1.0);
        frcsim::RigidBody::AerodynamicGeometry geometry;
        geometry.shape = frcsim::RigidBody::AerodynamicGeometry::Shape::kCylinder;
        geometry.radius_m = 0.1;
        geometry.cylinder_length_m = 0.5;

        body.setAerodynamicGeometry(geometry);
        body.setCylinderAxisLocal(frcsim::RigidBody::CylinderAxis::kX);

        const double area_end_on = body.dragReferenceAreaM2(frcsim::Vector3(10.0, 0.0, 0.0));
        const double area_side_on = body.dragReferenceAreaM2(frcsim::Vector3(0.0, 0.0, 10.0));

        const double expected_end_on = 3.14159265358979323846 * 0.1 * 0.1;
        const double expected_side_on = 2.0 * 0.1 * 0.5;

        assert(std::fabs(area_end_on - expected_end_on) < 1e-9);
        assert(std::fabs(area_side_on - expected_side_on) < 1e-9);
        std::cout << "  ✓ Cylinder axis convenience setter works\n";
    }

    // ===== Aerodynamics: Cylinder World Axis Convenience Setter =====
    {
        frcsim::RigidBody body(1.0);
        frcsim::RigidBody::AerodynamicGeometry geometry;
        geometry.shape = frcsim::RigidBody::AerodynamicGeometry::Shape::kCylinder;
        geometry.radius_m = 0.1;
        geometry.cylinder_length_m = 0.5;

        body.setAerodynamicGeometry(geometry);
        body.setOrientation(frcsim::Quaternion::fromAxisAngle(
            frcsim::Vector3::unitY(), 1.5707963267948966));
        body.setCylinderAxisWorld(frcsim::Vector3::unitX());

        const auto* configured = body.aerodynamicGeometry();
        assert(configured != nullptr);
        assert(std::fabs(configured->cylinder_axis_local.x) < 1e-9);
        assert(std::fabs(configured->cylinder_axis_local.y) < 1e-9);
        assert(std::fabs(configured->cylinder_axis_local.z - 1.0) < 1e-9);

        const double area_end_on = body.dragReferenceAreaM2(frcsim::Vector3(10.0, 0.0, 0.0));
        const double area_side_on = body.dragReferenceAreaM2(frcsim::Vector3(0.0, 0.0, 10.0));

        const double expected_end_on = 3.14159265358979323846 * 0.1 * 0.1;
        const double expected_side_on = 2.0 * 0.1 * 0.5;

        assert(std::fabs(area_end_on - expected_end_on) < 1e-9);
        assert(std::fabs(area_side_on - expected_side_on) < 1e-9);
        std::cout << "  ✓ Cylinder world-axis convenience setter works\n";
    }

    // ===== Aerodynamics: Invalid Drag Inputs Return Zero Force =====
    {
        const auto details = frcsim::Vector3::dragForceDetailed(
            frcsim::Vector3(10.0, 0.0, 0.0), -0.47, 0.01);

        assert(!details.valid);
        assert(details.force.isZero());
        std::cout << "  ✓ Invalid drag inputs fail closed\n";
    }

    // ===== Aerodynamics: Disabled Test =====
    {
        frcsim::PhysicsConfig config;
        config.fixed_dt_s = 0.01;
        config.enable_collision_detection = false;
        config.enable_joint_constraints = false;
        config.enable_gravity = false;
        config.enable_aerodynamics = false;
        config.linear_damping_per_s = 0.0;

        frcsim::PhysicsWorld world(config);
        frcsim::RigidBody& body = world.createBody(1.0);

        body.setLinearVelocity(frcsim::Vector3(10.0, 0.0, 0.0));

        for (int i = 0; i < 100; ++i) {
            world.step();
        }

        assert(std::fabs(body.linearVelocity().x - 10.0) < 1e-6);
        std::cout << "  ✓ Aerodynamics can be disabled\n";
    }

    // ===== Magnus Effect Simulation Test =====
    {
        frcsim::PhysicsConfig config;
        config.fixed_dt_s = 0.01;
        config.enable_collision_detection = false;
        config.enable_joint_constraints = false;
        config.enable_gravity = false;
        config.enable_aerodynamics = true;
        config.linear_damping_per_s = 0.0;
        config.angular_damping_per_s = 0.0;

        frcsim::PhysicsWorld world(config);
        frcsim::RigidBody& body = world.createBody(0.05);

        body.setLinearVelocity(frcsim::Vector3(10.0, 0.0, 0.0));
        body.setAngularVelocity(frcsim::Vector3(0.0, 50.0, 0.0));

        for (int i = 0; i < 50; ++i) {
            world.step();
        }

        assert(body.position().x > 0.0);
        std::cout << "  ✓ Magnus effect simulation runs\n";
    }

    std::cout << "✓ All force and aerodynamics tests passed!\n";
    return 0;
}
