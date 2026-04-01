#include <cassert>

#include "frcsim/aerodynamics/magnus_model.hpp"

int main() {
    frcsim::MagnusModel model(1.0e-3);
    const frcsim::Vector3 velocity(10.0, 0.0, 0.0);
    const frcsim::Vector3 spin(0.0, 0.0, 100.0);

    const frcsim::Vector3 force = model.computeForce(velocity, spin);

    // omega x v with spin +Z and velocity +X should produce +Y force.
    assert(force.y > 0.0);
    assert(force.x == 0.0);
    assert(force.z == 0.0);

    return 0;
}
