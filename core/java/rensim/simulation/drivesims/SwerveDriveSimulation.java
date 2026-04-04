package rensim.simulation.drivesims;

import frcsim_physics.RigidBody;
import frcsim_physics.SwerveModel;
import rensim.simulation.Pose2;

/**
 * Maple-style swerve drive simulation wrapper on top of RenSim rigid-body APIs.
 */
public final class SwerveDriveSimulation {
  private final DriveTrainSimulationConfig config;
  private final RigidBody rigidBody;
  private final SwerveModel model;

  /**
   * Creates a swerve simulation wrapper from config and rigid-body host.
   */
  public SwerveDriveSimulation(DriveTrainSimulationConfig config, RigidBody rigidBody) {
    this.config = config;
    this.rigidBody = rigidBody;
    this.model = new SwerveModel(config.trackLengthXMeters(), config.trackWidthYMeters(),
        config.maxWheelSpeedMps());
  }

  /**
   * Requests chassis velocity setpoint for this simulation tick.
   */
  public void runChassisSpeeds(double vxMps, double vyMps, double omegaRadPerSecond) {
    model.applyToBody(rigidBody, new SwerveModel.ChassisCommand(vxMps, vyMps, omegaRadPerSecond));
  }

  /**
   * Advances drivetrain-specific simulation state.
   */
  public void simulationSubTick(double dtSeconds) {
    rigidBody.advance(dtSeconds);
  }

  /**
   * Gets current simulated pose.
   */
  public Pose2 pose() {
    var p = rigidBody.positionMeters();
    return new Pose2(p.x(), p.y(), 0.0);
  }

  /**
   * Gets max modeled chassis linear speed.
   */
  public double maxLinearVelocityMps() {
    return config.maxWheelSpeedMps();
  }

  /**
   * Gets configured wheelbase radius proxy.
   */
  public double driveBaseRadiusMeters() {
    double halfL = config.trackLengthXMeters() * 0.5;
    double halfW = config.trackWidthYMeters() * 0.5;
    return Math.hypot(halfL, halfW);
  }
}