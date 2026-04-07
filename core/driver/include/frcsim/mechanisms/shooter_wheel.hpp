#pragma once

#include <algorithm>
#include <cmath>

namespace frcsim {

/** @brief Single-wheel shooter velocity model with optional closed-loop speed command behavior. */
class ShooterWheelSim {
  public:
	/** @brief DC motor approximation constants for shooter drive. */
	struct MotorConfig {
		// Motor free speed at nominal voltage.
		double free_speed_radps{600.0};

		// Stall torque at nominal voltage.
		double stall_torque_nm{2.0};

		// Approximate stall current for simple current limiting.
		double stall_current_a{120.0};

		// Applied supply voltage for normalization.
		double nominal_voltage_v{12.0};

		/**
		 * @brief Returns Falcon 500 aggregate motor constants.
		 * @param motor_count Number of mechanically coupled motors.
		 * @return MotorConfig scaled for the provided motor count.
		 */
		static MotorConfig falcon500(int motor_count = 1) {
			const int count = std::max(1, motor_count);
			MotorConfig cfg{};
			cfg.free_speed_radps = 668.0;
			cfg.stall_torque_nm = 4.69 * static_cast<double>(count);
			cfg.stall_current_a = 257.0 * static_cast<double>(count);
			cfg.nominal_voltage_v = 12.0;
			return cfg;
		}

		/**
		 * @brief Returns NEO V1.1 aggregate motor constants.
		 * @param motor_count Number of mechanically coupled motors.
		 * @return MotorConfig scaled for the provided motor count.
		 */
		static MotorConfig neoV1_1(int motor_count = 1) {
			const int count = std::max(1, motor_count);
			MotorConfig cfg{};
			cfg.free_speed_radps = 594.0;
			cfg.stall_torque_nm = 3.36 * static_cast<double>(count);
			cfg.stall_current_a = 166.0 * static_cast<double>(count);
			cfg.nominal_voltage_v = 12.0;
			return cfg;
		}

		/**
		 * @brief Returns Kraken X60 aggregate motor constants.
		 * @param motor_count Number of mechanically coupled motors.
		 * @return MotorConfig scaled for the provided motor count.
		 */
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

	/** @brief Shooter wheel inertial/friction and ball-coupling parameters. */
	struct WheelConfig {
		double radius_m{0.05};
		double inertia_kgm2{0.0025};
		double viscous_friction_nm_per_radps{0.0008};

		// Fraction of wheel tangential speed transferred to ball exit speed.
		double ball_coupling{0.88};
	};

	/** @brief Control inputs for open-loop or simple internal velocity-loop operation. */
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

	/**
	 * @brief Constructs a wheel simulator with explicit motor and wheel configuration.
	 * @param motor Motor configuration constants.
	 * @param wheel Wheel mechanical configuration.
	 */
	ShooterWheelSim(const MotorConfig& motor, const WheelConfig& wheel)
		: motor_(motor), wheel_(wheel) {}

	/** @brief Sets motor constants. @param motor New motor configuration. */
	void setMotorConfig(const MotorConfig& motor) { motor_ = motor; }
	/** @brief Sets wheel constants. @param wheel New wheel configuration. */
	void setWheelConfig(const WheelConfig& wheel) { wheel_ = wheel; }

	/** @brief Returns motor configuration. @return Immutable MotorConfig reference. */
	const MotorConfig& motorConfig() const { return motor_; }
	/** @brief Returns wheel configuration. @return Immutable WheelConfig reference. */
	const WheelConfig& wheelConfig() const { return wheel_; }

	/** @brief Forces internal angular speed state. @param omega_radps Wheel angular speed in rad/s. */
	void setAngularSpeedRadps(double omega_radps) { omega_radps_ = omega_radps; }
	/** @brief Returns current wheel angular speed. @return Angular speed in rad/s. */
	double angularSpeedRadps() const { return omega_radps_; }

	/** @brief Computes tangential speed at wheel perimeter. @return Surface speed in m/s. */
	double surfaceSpeedMps() const { return omega_radps_ * std::max(0.0, wheel_.radius_m); }

	/** @brief Estimates ball exit speed from current surface speed and coupling factor. @return Exit speed in m/s. */
	double estimatedExitVelocityMps() const {
		return std::max(0.0, surfaceSpeedMps() * std::max(0.0, wheel_.ball_coupling));
	}

	/**
	 * @brief Advances the shooter wheel state by one step.
	 * @param dt_s Timestep in seconds.
	 * @param control Control input for this step.
	 */
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
