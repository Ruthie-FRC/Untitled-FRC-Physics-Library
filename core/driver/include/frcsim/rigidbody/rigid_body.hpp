// Copyright (c) RenSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

#include <algorithm>
#include <cmath>
#include <optional>

#include "frcsim/math/integrators.hpp"
#include "frcsim/math/matrix.hpp"
#include "frcsim/math/quaternion.hpp"
#include "frcsim/math/vector.hpp"
#include "frcsim/rigidbody/body_flags.hpp"
#include "frcsim/rigidbody/material.hpp"

namespace frcsim {

enum class IntegrationMethod {
  kSemiImplicitEuler,
  kExplicitEuler,
  kRK2,
};

class RigidBody {
 public:
  enum class CylinderAxis {
    kX,
    kY,
    kZ,
  };

  struct AerodynamicGeometry {
    enum class Shape {
      kCustom,
      kSphere,
      kBox,
      kCylinder,
    };

    Shape shape{Shape::kCustom};
    double reference_area_m2{0.0};
    double radius_m{0.0};
    Vector3 box_dimensions_m{0.0, 0.0, 0.0};
    double cylinder_length_m{0.0};
    Vector3 cylinder_axis_local{0.0, 0.0, 1.0};
  };

  explicit RigidBody(double mass_kg = 1.0) { setMassKg(mass_kg); }

  double massKg() const { return mass_kg_; }
  double inverseMass() const { return inv_mass_; }
  void setMassKg(double mass_kg) {
    mass_kg_ = (mass_kg > 0.0) ? mass_kg : 1.0;
    inv_mass_ = 1.0 / mass_kg_;
  }

  const Vector3& position() const { return position_m_; }
  void setPosition(const Vector3& position_m) { position_m_ = position_m; }

  const Quaternion& orientation() const { return orientation_; }
  void setOrientation(const Quaternion& orientation) {
    orientation_ = orientation;
  }

  const Vector3& linearVelocity() const { return linear_velocity_mps_; }
  void setLinearVelocity(const Vector3& velocity_mps) {
    linear_velocity_mps_ = velocity_mps;
  }

  const Vector3& angularVelocity() const { return angular_velocity_radps_; }
  void setAngularVelocity(const Vector3& angular_velocity_radps) {
    angular_velocity_radps_ = angular_velocity_radps;
  }

  const Matrix3& bodyInertiaTensor() const { return body_inertia_tensor_; }
  void setBodyInertiaTensor(const Matrix3& inertia) {
    body_inertia_tensor_ = inertia;
    inv_body_inertia_tensor_ = inertia.inverse();
  }

  BodyFlags& flags() { return flags_; }
  const BodyFlags& flags() const { return flags_; }

  void setMaterial(const Material& material) { material_ = material; }
  const Material* material() const {
    return material_ ? &(*material_) : nullptr;
  }

  void setAerodynamicGeometry(const AerodynamicGeometry& geometry) {
    aerodynamic_geometry_ = geometry;
  }
  const AerodynamicGeometry* aerodynamicGeometry() const {
    return aerodynamic_geometry_ ? &(*aerodynamic_geometry_) : nullptr;
  }

  void setCylinderAxisLocal(CylinderAxis axis) {
    if (!aerodynamic_geometry_) {
      aerodynamic_geometry_ = AerodynamicGeometry{};
    }

    switch (axis) {
      case CylinderAxis::kX:
        aerodynamic_geometry_->cylinder_axis_local = Vector3::unitX();
        break;
      case CylinderAxis::kY:
        aerodynamic_geometry_->cylinder_axis_local = Vector3::unitY();
        break;
      case CylinderAxis::kZ:
      default:
        aerodynamic_geometry_->cylinder_axis_local = Vector3::unitZ();
        break;
    }
  }

  void setCylinderAxisWorld(const Vector3& axis_world) {
    if (!aerodynamic_geometry_) {
      aerodynamic_geometry_ = AerodynamicGeometry{};
    }

    Vector3 axis_local = orientation_.inverse().rotate(axis_world).normalized();
    if (axis_local.isZero()) {
      axis_local = Vector3::unitZ();
    }

    aerodynamic_geometry_->cylinder_axis_local = axis_local;
  }

  double dragReferenceAreaM2(const Vector3& velocity_world) const {
    if (!aerodynamic_geometry_)
      return 0.0;

    const AerodynamicGeometry& geometry = *aerodynamic_geometry_;
    if (geometry.reference_area_m2 > 0.0)
      return geometry.reference_area_m2;

    switch (geometry.shape) {
      case AerodynamicGeometry::Shape::kSphere: {
        const double radius_m = std::max(0.0, geometry.radius_m);
        return 3.14159265358979323846 * radius_m * radius_m;
      }
      case AerodynamicGeometry::Shape::kBox: {
        if (geometry.box_dimensions_m.x <= 0.0 ||
            geometry.box_dimensions_m.y <= 0.0 ||
            geometry.box_dimensions_m.z <= 0.0) {
          return 0.0;
        }

        Vector3 velocity_direction = velocity_world.isZero()
                                         ? Vector3::unitX()
                                         : velocity_world.normalized();
        velocity_direction = orientation_.inverse().rotate(velocity_direction);

        const Vector3 dims = geometry.box_dimensions_m;
        return std::abs(velocity_direction.x) * dims.y * dims.z +
               std::abs(velocity_direction.y) * dims.x * dims.z +
               std::abs(velocity_direction.z) * dims.x * dims.y;
      }
      case AerodynamicGeometry::Shape::kCylinder: {
        const double radius_m = std::max(0.0, geometry.radius_m);
        const double length_m = std::max(0.0, geometry.cylinder_length_m);
        if (radius_m <= 0.0 || length_m <= 0.0) {
          return 0.0;
        }

        const Vector3 velocity_direction_world =
            velocity_world.isZero() ? Vector3::unitX()
                                    : velocity_world.normalized();
        Vector3 cylinder_axis_local = geometry.cylinder_axis_local.normalized();
        if (cylinder_axis_local.isZero()) {
          cylinder_axis_local = Vector3::unitZ();
        }

        const Vector3 velocity_direction_local =
            orientation_.inverse().rotate(velocity_direction_world);
        const double axis_alignment =
            std::abs(velocity_direction_local.dot(cylinder_axis_local));
        const double side_alignment =
            std::sqrt(std::max(0.0, 1.0 - axis_alignment * axis_alignment));

        const double endcap_area_m2 =
            3.14159265358979323846 * radius_m * radius_m * axis_alignment;
        const double side_area_m2 = 2.0 * radius_m * length_m * side_alignment;
        return endcap_area_m2 + side_area_m2;
      }
      case AerodynamicGeometry::Shape::kCustom:
      default:
        return 0.0;
    }
  }

  void applyForce(const Vector3& force_n) { accumulated_force_n_ += force_n; }

  void applyForceAtPoint(const Vector3& force_n, const Vector3& world_point_m) {
    applyForce(force_n);
    const Vector3 r = world_point_m - position_m_;
    accumulated_torque_nm_ += r.cross(force_n);
  }

  void clearAccumulators() {
    accumulated_force_n_ = Vector3::zero();
    accumulated_torque_nm_ = Vector3::zero();
  }

  void integrate(double dt_s, IntegrationMethod method,
                 const Vector3& gravity_mps2, double linear_damping_per_s,
                 double angular_damping_per_s) {
    if (flags_.is_kinematic) {
      clearAccumulators();
      return;
    }

    Vector3 total_force = accumulated_force_n_;
    if (flags_.enable_gravity) {
      total_force += gravity_mps2 * mass_kg_;
    }

    const Vector3 linear_accel = total_force * inv_mass_;
    switch (method) {
      case IntegrationMethod::kExplicitEuler:
        Integrator::integrateLinearExplicit(position_m_, linear_velocity_mps_,
                                            linear_accel, dt_s);
        break;
      case IntegrationMethod::kRK2:
        Integrator::integrateLinearRK2(position_m_, linear_velocity_mps_,
                                       linear_accel, dt_s);
        break;
      case IntegrationMethod::kSemiImplicitEuler:
      default:
        Integrator::integrateLinear(position_m_, linear_velocity_mps_,
                                    linear_accel, dt_s);
        break;
    }

    const double linear_damp = std::max(0.0, 1.0 - linear_damping_per_s * dt_s);
    linear_velocity_mps_ *= linear_damp;

    const Vector3 angular_accel =
        inv_body_inertia_tensor_ * accumulated_torque_nm_;
    Integrator::integrateAngularVelocity(angular_velocity_radps_, angular_accel,
                                         dt_s);
    Integrator::integrateAngular(orientation_, angular_velocity_radps_, dt_s);

    const double angular_damp =
        std::max(0.0, 1.0 - angular_damping_per_s * dt_s);
    angular_velocity_radps_ *= angular_damp;

    clearAccumulators();
  }

 private:
  double mass_kg_{1.0};
  double inv_mass_{1.0};

  Vector3 position_m_{};
  Quaternion orientation_{};
  Vector3 linear_velocity_mps_{};
  Vector3 angular_velocity_radps_{};

  Vector3 accumulated_force_n_{};
  Vector3 accumulated_torque_nm_{};

  Matrix3 body_inertia_tensor_{Matrix3::identity()};
  Matrix3 inv_body_inertia_tensor_{Matrix3::identity()};

  BodyFlags flags_{};
  std::optional<Material> material_{};
  std::optional<AerodynamicGeometry> aerodynamic_geometry_{};
};

}  // namespace frcsim
