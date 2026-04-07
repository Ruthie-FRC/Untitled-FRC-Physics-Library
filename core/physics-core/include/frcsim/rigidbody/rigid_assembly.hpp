#pragma once

#include <memory>
#include <vector>

#include "frcsim/joints/joint_base.hpp"
#include "frcsim/rigidbody/rigid_body.hpp"

namespace frcsim {

class RevoluteJoint;
class PrismaticJoint;
class FixedJoint;

class RigidAssembly {
  public:
    RigidAssembly() = default;

    /// Add a body to this assembly (takes ownership)
    RigidBody* addBody(double mass_kg);

    /// Retrieve bodies
    std::vector<RigidBody>& bodies() { return bodies_; }
    const std::vector<RigidBody>& bodies() const { return bodies_; }

    /// Create joints between bodies in this assembly
    RevoluteJoint* addRevoluteJoint(size_t body_a_idx, size_t body_b_idx, const Vector3& axis_local);
    PrismaticJoint* addPrismaticJoint(size_t body_a_idx, size_t body_b_idx, const Vector3& axis_local);
    FixedJoint* addFixedJoint(size_t body_a_idx, size_t body_b_idx);

    /// Joint management
    std::vector<std::shared_ptr<JointBase>>& joints() { return joints_; }
    const std::vector<std::shared_ptr<JointBase>>& joints() const { return joints_; }

    /// Mark one body as the "root" or primary reference (no physical meaning, just organization)
    void setRootBodyIndex(size_t idx);
    size_t rootBodyIndex() const { return root_body_idx_; }

    /// Get total assembly mass
    double totalMass() const;

    /// Compute assembly center of mass in world space
    Vector3 centerOfMass() const;

    /// Disable/enable all constraints in assembly
    void enableConstraints(bool enable);
    void solveConstraints(double dt_s, int iterations);

  private:
    std::vector<RigidBody> bodies_;
    std::vector<std::shared_ptr<JointBase>> joints_;
    size_t root_body_idx_{0};
};

}  // namespace frcsim
