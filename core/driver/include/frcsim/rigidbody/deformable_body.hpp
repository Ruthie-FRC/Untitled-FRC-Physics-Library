// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

#include <vector>

#include "frcsim/math/matrix.hpp"
#include "frcsim/math/quaternion.hpp"
#include "frcsim/math/vector.hpp"
#include "frcsim/rigidbody/rigid_body.hpp"

namespace frcsim {

/**
 * @brief Rigid body with optional deformable mesh dynamics overlay.
 *
 * Extends rigid body behavior with per-node deformation tracking in local
 * coordinates.
 * @details This is intentionally overkill for current use cases, but it keeps
 * the future flex-body integration path explicit.
 * TODO: Implement deformation solving during simulation substeps.
 */
class DeformableBody {
 public:
  /**
   * @brief Constructs a rigid-body-based deformable body.
   * @param mass_kg Body mass in kilograms.
   */
  explicit DeformableBody(double mass_kg = 1.0) : rigid_base_(mass_kg) {}

  /** @brief Immutable access to underlying rigid body state and properties. */
  const RigidBody& rigidBase() const { return rigid_base_; }
  /** @brief Mutable access to underlying rigid body. */
  RigidBody& rigidBase() { return rigid_base_; }

  /**
   * @brief Enables or disables deformation solving.
   * @param enable Whether deformation dynamics are active.
   */
  void enableDeformation(bool enable) { enable_deformation_ = enable; }
  /** @brief Returns whether deformation is currently enabled. */
  bool isDeformationEnabled() const { return enable_deformation_; }

  /**
   * @brief Sets the bending stiffness for deformation resistance.
   * @param stiffness_npm Stiffness in newtons per meter.
   */
  void setBendStiffness(double stiffness_npm) {
    bend_stiffness_npm_ = stiffness_npm;
  }
  /** @brief Returns bending stiffness in newtons per meter. */
  double bendStiffness() const { return bend_stiffness_npm_; }

  /**
   * @brief Sets the warp damping coefficient for deformation energy dissipation.
   * @param damping_nspm Damping in newton-seconds per meter.
   */
  void setWarpDamping(double damping_nspm) {
    warp_damping_nspm_ = damping_nspm;
  }
  /** @brief Returns warp damping in newton-seconds per meter. */
  double warpDamping() const { return warp_damping_nspm_; }

  /**
   * @brief Immutable access to local-frame deformation node positions.
   * @return Const reference to vector of node positions in body-local coordinates.
   * @note TODO: Populate with actual deformation mesh nodes during application setup.
   */
  const std::vector<Vector3>& deformationNodes() const {
    return deformation_nodes_local_;
  }
  /**
   * @brief Mutable access to local-frame deformation node positions.
   * @return Reference to vector of node positions in body-local coordinates.
   */
  std::vector<Vector3>& deformationNodes() { return deformation_nodes_local_; }

  /**
   * @brief Immutable access to deformation node velocities.
   * @return Const reference to velocity vector for bending dynamics.
   */
  const std::vector<Vector3>& deformationVelocities() const {
    return deformation_velocities_;
  }
  /**
   * @brief Mutable access to deformation node velocities.
   * @return Reference to velocity vector for bending dynamics.
   */
  std::vector<Vector3>& deformationVelocities() {
    return deformation_velocities_;
  }

 private:
  RigidBody rigid_base_;
  bool enable_deformation_{false};
  double bend_stiffness_npm_{100.0};
  double warp_damping_nspm_{10.0};

  // TODO: Populate with actual deformation mesh nodes during application setup.
  std::vector<Vector3> deformation_nodes_local_;
  std::vector<Vector3> deformation_velocities_;
};

}  // namespace frcsim
