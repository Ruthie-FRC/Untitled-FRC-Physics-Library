// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
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
 * @brief Simulated rigid body with translational/angular dynamics and optional aero metadata.
 *
 * This type stores canonical rigid-body state (pose, linear velocity, angular
 * velocity, mass, inertia) and supports force/torque accumulation followed by
 * fixed-step integration.
 *
 * Key behavior:
 * - Forces accumulate until @ref integrate is called.
 * - Gravity application is controlled by per-body flags and world config.
 * - Optional material metadata informs contact response in @ref PhysicsWorld.
 * - Optional aerodynamic geometry supports runtime drag-area estimation.
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

  /**
   * @brief Geometric metadata used to estimate drag reference area.
   *
   * Area estimation can be explicit (via @ref reference_area_m2) or inferred from
   * a primitive shape. Inferred area depends on velocity direction and orientation
   * for non-spherical shapes.
   */
  struct AerodynamicGeometry {
    /** @brief Primitive shape model used for reference-area estimation. */
    enum class Shape {
      /** @brief Use only explicit caller-provided area (no shape inference). */
      kCustom,
      /** @brief Sphere model parameterized by @ref radius_m. */
      kSphere,
      /** @brief Box model parameterized by @ref box_dimensions_m. */
      kBox,
      /** @brief Cylinder model parameterized by radius, length, and axis. */
      kCylinder,
    };

    /** @brief Selected shape model. */
    Shape shape{Shape::kCustom};

    /**
     * @brief Explicit area override in square meters.
     *
     * When > 0, this value bypasses shape-based area estimation.
     */
    double reference_area_m2{0.0};

    /** @brief Sphere/cylinder radius in meters. */
    double radius_m{0.0};

    /** @brief Box dimensions (x, y, z) in meters. */
    Vector3 box_dimensions_m{0.0, 0.0, 0.0};

    /** @brief Cylinder length in meters along @ref cylinder_axis_local. */
    double cylinder_length_m{0.0};

    /** @brief Cylinder principal axis expressed in local body coordinates. */
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
  /**
   * @brief Sets world-space position in meters.
   * @param position_m World-space position in meters.
   */
  void setPosition(const Vector3& position_m) { position_m_ = position_m; }
  /**
   * @brief Sets world-space position in meters from scalar components.
   * @param x_m World-space x position in meters.
   * @param y_m World-space y position in meters.
   * @param z_m World-space z position in meters.
   */
  void setPosition(double x_m, double y_m, double z_m) {
    setPosition(Vector3{x_m, y_m, z_m});
  }
  /**
   * @brief Gets world-space position in meters via scalar output pointers.
   * @param x_m Optional output pointer for world-space x position in meters.
   * @param y_m Optional output pointer for world-space y position in meters.
   * @param z_m Optional output pointer for world-space z position in meters.
   */
  void position(double* x_m, double* y_m, double* z_m) const {
    if (x_m) {
      *x_m = position_m_.x;
    }
    if (y_m) {
      *y_m = position_m_.y;
    }
    if (z_m) {
      *z_m = position_m_.z;
    }
  }

  /** @brief Returns body orientation as a unit quaternion. */
  const Quaternion& orientation() const { return orientation_; }
  /**
   * @brief Sets body orientation quaternion.
   * @param orientation New world-space body orientation quaternion.
   */
  void setOrientation(const Quaternion& orientation) {
    orientation_ = orientation;
  }

  /** @brief Returns world linear velocity in meters per second. */
  const Vector3& linearVelocity() const { return linear_velocity_mps_; }
  /**
   * @brief Sets world linear velocity in meters per second.
   * @param velocity_mps World-space linear velocity in meters per second.
   */
  void setLinearVelocity(const Vector3& velocity_mps) {
    linear_velocity_mps_ = velocity_mps;
  }
  /**
   * @brief Sets world linear velocity in meters per second from scalars.
   * @param vx_mps World-space x velocity in meters per second.
   * @param vy_mps World-space y velocity in meters per second.
   * @param vz_mps World-space z velocity in meters per second.
   */
  void setLinearVelocity(double vx_mps, double vy_mps, double vz_mps) {
    setLinearVelocity(Vector3{vx_mps, vy_mps, vz_mps});
  }
  /**
   * @brief Gets world linear velocity in meters per second via pointers.
   * @param vx_mps Optional output pointer for world-space x velocity in m/s.
   * @param vy_mps Optional output pointer for world-space y velocity in m/s.
   * @param vz_mps Optional output pointer for world-space z velocity in m/s.
   */
  void linearVelocity(double* vx_mps, double* vy_mps, double* vz_mps) const {
    if (vx_mps) {
      *vx_mps = linear_velocity_mps_.x;
    }
    if (vy_mps) {
      *vy_mps = linear_velocity_mps_.y;
    }
    if (vz_mps) {
      *vz_mps = linear_velocity_mps_.z;
    }
  }

  /** @brief Returns world angular velocity in radians per second. */
  const Vector3& angularVelocity() const { return angular_velocity_radps_; }
  /**
   * @brief Sets world angular velocity in radians per second.
   * @param angular_velocity_radps World-space angular velocity in radians per
   * second.
   */
  void setAngularVelocity(const Vector3& angular_velocity_radps) {
    angular_velocity_radps_ = angular_velocity_radps;
  }
  /**
   * @brief Sets world angular velocity in radians per second from scalars.
   * @param wx_radps World-space angular velocity x in radians per second.
   * @param wy_radps World-space angular velocity y in radians per second.
   * @param wz_radps World-space angular velocity z in radians per second.
   */
  void setAngularVelocity(double wx_radps, double wy_radps, double wz_radps) {
    setAngularVelocity(Vector3{wx_radps, wy_radps, wz_radps});
  }
  /**
   * @brief Gets world angular velocity in radians per second via pointers.
   * @param wx_radps Optional output pointer for world-space angular velocity x.
   * @param wy_radps Optional output pointer for world-space angular velocity y.
   * @param wz_radps Optional output pointer for world-space angular velocity z.
   */
  void angularVelocity(double* wx_radps, double* wy_radps, double* wz_radps) const {
    if (wx_radps) {
      *wx_radps = angular_velocity_radps_.x;
    }
    if (wy_radps) {
      *wy_radps = angular_velocity_radps_.y;
    }
    if (wz_radps) {
      *wz_radps = angular_velocity_radps_.z;
    }
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

  /**
   * @brief Sets numeric material identifier used by world interaction tables.
   * @param material_id Application-defined material id.
   */
  void setMaterialId(std::int32_t material_id) { material_id_ = material_id; }

  /**
   * @brief Gets numeric material identifier used by world interaction tables.
   * @return Material id currently assigned to this body.
   */
  std::int32_t materialId() const { return material_id_; }

  /**
   * @brief Sets broad-phase collision layer bits for this body.
   * @param layer_bits Layer membership bitset.
   */
  void setCollisionLayer(std::uint32_t layer_bits) { collision_layer_bits_ = layer_bits; }

  /**
   * @brief Gets broad-phase collision layer bits.
   * @return Layer membership bitset.
   */
  std::uint32_t collisionLayer() const { return collision_layer_bits_; }

  /**
   * @brief Sets broad-phase collision mask bits for this body.
   * @param mask_bits Interaction acceptance bitset.
   */
  void setCollisionMask(std::uint32_t mask_bits) { collision_mask_bits_ = mask_bits; }

  /**
   * @brief Gets broad-phase collision mask bits.
   * @return Interaction acceptance bitset.
   */
  std::uint32_t collisionMask() const { return collision_mask_bits_; }

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
   * @brief Sets cylinder axis from world-space scalar direction components.
   * @param x World-space x direction component.
   * @param y World-space y direction component.
   * @param z World-space z direction component.
   */
  void setCylinderAxisWorld(double x, double y, double z) {
    setCylinderAxisWorld(Vector3{x, y, z});
  }

  /**
   * @brief Computes effective drag reference area for current geometry and motion direction.
   * @param velocity_world World-space velocity vector used to infer projected area.
   * @return Effective reference area in square meters.
   *
   * Behavior by geometry mode:
   * - No geometry configured: returns 0.
   * - Explicit reference area > 0: returns explicit value.
   * - Sphere: returns pi * r^2.
   * - Box: estimates projected area from local velocity direction and face areas.
   * - Cylinder: blends end-cap and side projected areas using axis alignment.
   *
   * For zero velocity vectors, a stable fallback direction is used to avoid NaN
   * behavior during normalization.
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
   * @brief Adds force at center of mass in newtons from scalar components.
   * @param fx_n World-space x force in newtons.
   * @param fy_n World-space y force in newtons.
   * @param fz_n World-space z force in newtons.
   */
  void applyForce(double fx_n, double fy_n, double fz_n) {
    applyForce(Vector3{fx_n, fy_n, fz_n});
  }

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

  /**
   * @brief Adds force at world-space point from scalar components.
    * @param fx_n World-space x force in newtons.
    * @param fy_n World-space y force in newtons.
    * @param fz_n World-space z force in newtons.
    * @param px_m World-space x application point in meters.
    * @param py_m World-space y application point in meters.
    * @param pz_m World-space z application point in meters.
   */
  void applyForceAtPoint(double fx_n, double fy_n, double fz_n, double px_m,
                         double py_m, double pz_m) {
    applyForceAtPoint(Vector3{fx_n, fy_n, fz_n}, Vector3{px_m, py_m, pz_m});
  }

  /** @brief Clears accumulated force and torque buffers. */
  void clearAccumulators() {
    accumulated_force_n_ = Vector3::zero();
    accumulated_torque_nm_ = Vector3::zero();
  }

  /**
   * @brief Advances translational and rotational state by one fixed timestep.
   * @param dt_s Timestep duration in seconds.
   * @param method Translation integration strategy.
   * @param gravity_mps2 World-space gravity acceleration in m/s^2.
   * @param linear_damping_per_s Linear damping coefficient in 1/s.
   * @param angular_damping_per_s Angular damping coefficient in 1/s.
   *
   * Integration sequence:
   * 1. Early-out for kinematic bodies (accumulators are still cleared).
   * 2. Sum accumulated forces and optional gravity contribution.
   * 3. Integrate linear state using selected method.
   * 4. Apply linear damping attenuation.
   * 5. Integrate angular velocity and orientation.
   * 6. Apply angular damping attenuation.
   * 7. Clear accumulated force/torque buffers.
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
  std::int32_t material_id_{0};
  std::uint32_t collision_layer_bits_{0xFFFFFFFFu};
  std::uint32_t collision_mask_bits_{0xFFFFFFFFu};
  std::optional<AerodynamicGeometry> aerodynamic_geometry_{};
};

}  // namespace frcsim
