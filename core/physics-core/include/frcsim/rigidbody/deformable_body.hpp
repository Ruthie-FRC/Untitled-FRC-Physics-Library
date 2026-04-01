#pragma once

#include <vector>

#include "frcsim/math/matrix.hpp"
#include "frcsim/math/quaternion.hpp"
#include "frcsim/math/vector.hpp"
#include "frcsim/rigidbody/rigid_body.hpp"

namespace frcsim {

class DeformableBody {
  public:
    explicit DeformableBody(double mass_kg = 1.0) : rigid_base_(mass_kg) {}

    const RigidBody& rigidBase() const { return rigid_base_; }
    RigidBody& rigidBase() { return rigid_base_; }

    void enableDeformation(bool enable) { enable_deformation_ = enable; }
    bool isDeformationEnabled() const { return enable_deformation_; }

    void setBendStiffness(double stiffness_npm) { bend_stiffness_npm_ = stiffness_npm; }
    double bendStiffness() const { return bend_stiffness_npm_; }

    void setWarpDamping(double damping_nspm) { warp_damping_nspm_ = damping_nspm; }
    double warpDamping() const { return warp_damping_nspm_; }

    // Deformation nodes in local body coordinates (TODO: populate and manage during simulation).
    const std::vector<Vector3>& deformationNodes() const { return deformation_nodes_local_; }
    std::vector<Vector3>& deformationNodes() { return deformation_nodes_local_; }

    // Deformation velocity per node (for bend dynamics).
    const std::vector<Vector3>& deformationVelocities() const { return deformation_velocities_; }
    std::vector<Vector3>& deformationVelocities() { return deformation_velocities_; }

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
