#pragma once

#include <algorithm>
#include <cmath>

namespace frcsim {

class ShooterWheelSim {
  public:
	struct MotorConfig {
		// Motor free speed at nominal voltage.
		double free_speed_radps{600.0};

		// Stall torque at nominal voltage.
		double stall_torque_nm{2.0};

		// Approximate stall current for simple current limiting.
		double stall_current_a{120.0};

		// Applied supply voltage for normalization.
		double nominal_voltage_v{12.0};

		static MotorConfig falcon500(int motor_count = 1) {
			const int count = std::max(1, motor_count);
			MotorConfig cfg{};
			cfg.free_speed_radps = 668.0;
			cfg.stall_torque_nm = 4.69 * static_cast<double>(count);
			cfg.stall_current_a = 257.0 * static_cast<double>(count);
			cfg.nominal_voltage_v = 12.0;
			return cfg;
		}

		static MotorConfig neoV1_1(int motor_count = 1) {
			const int count = std::max(1, motor_count);
			MotorConfig cfg{};
			cfg.free_speed_radps = 594.0;
			cfg.stall_torque_nm = 3.36 * static_cast<double>(count);
			cfg.stall_current_a = 166.0 * static_cast<double>(count);
			cfg.nominal_voltage_v = 12.0;
			return cfg;
		}

		static MotorConfig krakenX60(int motor_count = 1) {
			const int count = std::max(1, motor_count);
			MotorConfig cfg{};
			cfg.free_speed_radps = 628.0;
			cfg.stall_torque_nm = 7.09 * static_cast<double>(count);
			cfg.stall_current_a = 366.0 * static_cast<double>(count);
			cfg.nominal_voltage_v = 12.0;
			return cfg;
		}
	};

	struct WheelConfig {
		double radius_m{0.05};
		double inertia_kgm2{0.0025};
		double viscous_friction_nm_per_radps{0.0008};

		// Fraction of wheel tangential speed transferred to ball exit speed.
		double ball_coupling{0.88};
	};

	struct ControlInput {
		// Open-loop voltage command.
		double command_voltage_v{0.0};

		// If true, internal velocity loop maps target speed to voltage.
		bool velocity_closed_loop{false};
		double target_speed_radps{0.0};
		double velocity_kp{0.03};

		// Optional stator current clamp, <=0 disables limiting.
		double current_limit_a{0.0};

		// Voltage needed to overcome static friction at very low speeds.
		double friction_voltage_v{0.0};
	};

	ShooterWheelSim() = default;

	ShooterWheelSim(const MotorConfig& motor, const WheelConfig& wheel)
		: motor_(motor), wheel_(wheel) {}

	void setMotorConfig(const MotorConfig& motor) { motor_ = motor; }
	void setWheelConfig(const WheelConfig& wheel) { wheel_ = wheel; }

	const MotorConfig& motorConfig() const { return motor_; }
	const WheelConfig& wheelConfig() const { return wheel_; }

	void setAngularSpeedRadps(double omega_radps) { omega_radps_ = omega_radps; }
	double angularSpeedRadps() const { return omega_radps_; }

	// Tangential speed at the wheel perimeter.
	double surfaceSpeedMps() const { return omega_radps_ * std::max(0.0, wheel_.radius_m); }

	// Estimated scalar exit speed for a ball leaving this wheel.
	double estimatedExitVelocityMps() const {
		return std::max(0.0, surfaceSpeedMps() * std::max(0.0, wheel_.ball_coupling));
	}

	void step(double dt_s, const ControlInput& control) {
		if (dt_s <= 0.0) {
			return;
		}

		const double effective_nominal_v = std::max(1e-9, motor_.nominal_voltage_v);
		const double effective_free_speed = std::max(1e-9, motor_.free_speed_radps);
		const double kt = motor_.stall_torque_nm / effective_nominal_v;

		double voltage = control.command_voltage_v;
		if (control.velocity_closed_loop) {
			const double error = control.target_speed_radps - omega_radps_;
			voltage = control.velocity_kp * error;
		}
		voltage = std::clamp(voltage, -effective_nominal_v, effective_nominal_v);

		if (std::abs(omega_radps_) < 1e-3) {
			const double v_deadband = std::max(0.0, control.friction_voltage_v);
			if (std::abs(voltage) <= v_deadband) {
				voltage = 0.0;
			} else {
				voltage = std::copysign(std::max(0.0, std::abs(voltage) - v_deadband), voltage);
			}
		}

		// Linearized DC motor: omega = free_speed*(1 - tau/stall_torque) for given normalized voltage.
		const double voltage_scale = voltage / effective_nominal_v;
		const double no_load_speed = effective_free_speed * voltage_scale;
		const double speed_error = no_load_speed - omega_radps_;
		double available_torque = motor_.stall_torque_nm * speed_error / effective_free_speed;

		if (control.current_limit_a > 0.0 && motor_.stall_current_a > 0.0) {
			const double torque_per_amp = motor_.stall_torque_nm / motor_.stall_current_a;
			const double max_torque_from_limit = torque_per_amp * control.current_limit_a;
			available_torque = std::clamp(available_torque, -max_torque_from_limit, max_torque_from_limit);
		}

		const double friction_torque = wheel_.viscous_friction_nm_per_radps * omega_radps_;
		const double net_torque = available_torque - friction_torque;

		const double inertia = std::max(1e-9, wheel_.inertia_kgm2);
		const double alpha_radps2 = net_torque / inertia;
		omega_radps_ += alpha_radps2 * dt_s;
	}

  private:
	MotorConfig motor_{};
	WheelConfig wheel_{};
	double omega_radps_{0.0};
};

}  // namespace frcsim
