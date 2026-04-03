#pragma once

#include <cstdint>

namespace rensim {

struct Vec3 {
	double x{0.0};
	double y{0.0};
	double z{0.0};
};

class PhysicsWorld {
 public:
	PhysicsWorld(double fixed_dt_seconds = 0.01, bool enable_gravity = true);
	~PhysicsWorld();

	PhysicsWorld(const PhysicsWorld&) = delete;
	PhysicsWorld& operator=(const PhysicsWorld&) = delete;

	PhysicsWorld(PhysicsWorld&& other) noexcept;
	PhysicsWorld& operator=(PhysicsWorld&& other) noexcept;

	int createBody(double mass_kg);
	bool setBodyPosition(int body_index, const Vec3& position_m);
	bool setBodyLinearVelocity(int body_index, const Vec3& velocity_mps);
	bool setBodyGravityEnabled(int body_index, bool enabled);
	bool setGravity(const Vec3& gravity_mps2);

	bool step(int steps = 1);
	bool bodyPosition(int body_index, Vec3* out_position_m) const;
	bool bodyLinearVelocity(int body_index, Vec3* out_velocity_mps) const;

	bool valid() const { return handle_ != 0; }

 private:
	std::uint64_t handle_{0};
};

}  // namespace rensim
