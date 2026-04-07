#pragma once

namespace frcsim {

struct BodyFlags {
    // Apply gravity to this body
    bool enable_gravity{true};
    
    // Apply friction and contact forces
    bool enable_friction{true};
    
    // Enable collision detection and response for this body
    bool enable_collisions{true};
    
    // Enable kinematic constraints (joints, etc.) affecting this body
    bool enable_kinematic_constraints{true};
    
    // For deformable bodies: enable deformation/warping/bending
    bool enable_deformation{false};
    
    // Body is kinematic (controlled by external code, not physics)
    bool is_kinematic{false};
};

}  // namespace frcsim
