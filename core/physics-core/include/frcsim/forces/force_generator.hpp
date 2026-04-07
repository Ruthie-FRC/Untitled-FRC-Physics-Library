#pragma once

namespace frcsim {

class RigidBody;

class ForceGenerator {
	public:
		virtual ~ForceGenerator() = default;
		virtual void apply(RigidBody& body, double dt_s) const = 0;
};

}  // namespace frcsim
