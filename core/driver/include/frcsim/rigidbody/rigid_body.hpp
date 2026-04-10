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

/** @brief Integration strategy used for rigid body state updates. */
enum class IntegrationMethod {
  /** @brief Velocity-first (symplectic) Euler integration. */
  kSemiImplicitEuler,
  /** @brief Position-first explicit Euler integration. */
  kExplicitEuler,
  /** @brief Second-order midpoint (RK2) integration. */
  kRK2,
};

/**
 * @brief Simulated rigid body with linear/angular dynamics and optional drag geometry.
 */
class RigidBody {
 public:
  /** @brief Principal axis selection for cylindrical aerodynamic geometry. */
  enum class CylinderAxis {
    /** @brief Local +X axis. */
    kX,
    /** @brief Local +Y axis. */
    kY,
    /** @brief Local +Z axis. */
    kZ,
  };

  /** @brief Geometric data used to estimate drag reference area. */
  struct AerodynamicGeometry {
    /** @brief Shape model used for area estimation. */
    enum class Shape {
      /** @brief Caller-provided area only. */
      kCustom,
      /** @brief Sphere model using `radius_m`. */
      kSphere,
      /** @brief Box model using `box_dimensions_m`. */
      kBox,
      /** @brief Cylinder model using `radius_m`, `cylinder_length_m`, and axis. */
      kCylinder,
    };

    /** @brief Shape used for drag-area estimation. */
    Shape shape{Shape::kCustom};
    /** @brief Explicit reference area in square meters (overrides shape calc when > 0). */
    double reference_area_m2{0.0};
    /** @brief Sphere/cylinder radius in meters. */
    double radius_m{0.0};
    /** @brief Box dimensions in meters. */
    Vector3 box_dimensions_m{0.0, 0.0, 0.0};
    /** @brief Cylinder body length in meters. */
    double cylinder_length_m{0.0};
    /** @brief Cylinder axis in local body coordinates. */
    Vector3 cylinder_axis_local{0.0, 0.0, 1.0};
  };

  /**
   * @brief Constructs a rigid body with a sanitized positive mass.
   * @param mass_kg Body mass in kilograms.
   */
  explicit RigidBody(double mass_kg = 1.0) { setMassKg(mass_kg); }

  /** @brief Returns mass in kilograms. */
  double massKg() const { return mass_kg_; }
  /** @brief Returns inverse mass in 1/kg. */
  double inverseMass() const { return inv_mass_; }
  /**
   * @brief Sets mass in kilograms.
   * @note Non-positive inputs are clamped to 1.0 kg.
   */
  void setMassKg(double mass_kg) {
    mass_kg_ = (mass_kg > 0.0) ? mass_kg : 1.0;
    inv_mass_ = 1.0 / mass_kg_;
  }

  /** @brief Returns world position in meters. */
  const Vector3& position() const { return position_m_; }
  /** @brief Sets world position in meters. */
  void setPosition(const Vector3& position_m) { position_m_ = position_m; }

  /** @brief Returns body orientation as a unit quaternion. */
  const Quaternion& orientation() const { return orientation_; }
  /** @brief Sets body orientation quaternion. */
  void setOrientation(const Quaternion& orientation) {
    orientation_ = orientation;
  }

  /** @brief Returns world linear velocity in meters per second. */
  const Vector3& linearVelocity() const { return linear_velocity_mps_; }
  /** @brief Sets world linear velocity in meters per second. */
  void setLinearVelocity(const Vector3& velocity_mps) {
    linear_velocity_mps_ = velocity_mps;
  }

  /** @brief Returns world angular velocity in radians per second. */
  const Vector3& angularVelocity() const { return angular_velocity_radps_; }
  /** @brief Sets world angular velocity in radians per second. */
  void setAngularVelocity(const Vector3& angular_velocity_radps) {
    angular_velocity_radps_ = angular_velocity_radps;
  }

  /** @brief Returns inertia tensor in body frame. */
  const Matrix3& bodyInertiaTensor() const { return body_inertia_tensor_; }
  /** @brief Sets body-frame inertia tensor and cached inverse tensor. */
  void setBodyInertiaTensor(const Matrix3& inertia) {
    body_inertia_tensor_ = inertia;
    inv_body_inertia_tensor_ = inertia.inverse();
  }

  /** @brief Mutable access to runtime body flags. */
  BodyFlags& flags() { return flags_; }
  /** @brief Const access to runtime body flags. */
  const BodyFlags& flags() const { return flags_; }

  /** @brief Sets optional physical material properties. */
  void setMaterial(const Material& material) { material_ = material; }
  /** @brief Gets material if configured, otherwise returns null. */
  const Material* material() const {
    return material_ ? &(*material_) : nullptr;
  }

  /** @brief Sets optional aerodynamic geometry metadata. */
  void setAerodynamicGeometry(const AerodynamicGeometry& geometry) {
    aerodynamic_geometry_ = geometry;
  }
  /** @brief Gets aerodynamic geometry if configured, otherwise returns null. */
  const AerodynamicGeometry* aerodynamicGeometry() const {
    return aerodynamic_geometry_ ? &(*aerodynamic_geometry_) : nullptr;
  }

  /**
   * @brief Sets cylinder axis in local coordinates via canonical enum axis.
   */
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

  /**
   * @brief Sets cylinder axis from world-space direction.
    * @param axis_world World-space direction to convert into local body space.
   *
   * The provided axis is transformed into local body space and normalized.
   * Zero-length input falls back to local +Z.
   */
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

  /**
   * @brief Computes drag reference area for current geometry and motion direction.
   * @param velocity_world World-space velocity direction source.
   * @return Effective reference area in square meters.
   */
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

  /** @brief Adds force at center of mass in newtons. */
  void applyForce(const Vector3& force_n) { accumulated_force_n_ += force_n; }

  /**
   * @brief Adds force at world-space point, accumulating both force and torque.
   * @param force_n Applied force in newtons.
   * @param world_point_m Application point in world meters.
   */
  void applyForceAtPoint(const Vector3& force_n, const Vector3& world_point_m) {
    applyForce(force_n);
    const Vector3 r = world_point_m - position_m_;
    accumulated_torque_nm_ += r.cross(force_n);
  }

  /** @brief Clears accumulated force and torque buffers. */
  void clearAccumulators() {
    accumulated_force_n_ = Vector3::zero();
    accumulated_torque_nm_ = Vector3::zero();
  }

  /**
   * @brief Advances translational and rotational state by one timestep.
   * @param dt_s Step duration in seconds.
   * @param method Linear integration method.
    * @param gravity_mps2 World-space gravity acceleration vector in m/s^2.
   * @param linear_damping_per_s Linear damping coefficient in 1/s.
   * @param angular_damping_per_s Angular damping coefficient in 1/s.
   */
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
