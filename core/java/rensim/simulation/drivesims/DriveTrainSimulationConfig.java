package rensim.simulation.drivesims;

/**
 * Configuration for swerve-drive simulation wrappers.
 */
public final class DriveTrainSimulationConfig {
  private double robotMassKg = 54.0;
  private double bumperLengthXMeters = 0.91;
  private double bumperWidthYMeters = 0.91;
  private double trackLengthXMeters = 0.52;
  private double trackWidthYMeters = 0.52;
  private double maxWheelSpeedMps = 5.5;

  /**
   * Returns a default FRC-like drivetrain config.
   */
  public static DriveTrainSimulationConfig defaults() {
    return new DriveTrainSimulationConfig();
  }

  public DriveTrainSimulationConfig withRobotMassKg(double robotMassKg) {
    if (!(robotMassKg > 0.0)) {
      throw new IllegalArgumentException("robotMassKg must be > 0");
    }
    this.robotMassKg = robotMassKg;
    return this;
  }

  public DriveTrainSimulationConfig withBumperDimensions(double lengthXMeters, double widthYMeters) {
    if (!(lengthXMeters > 0.0) || !(widthYMeters > 0.0)) {
      throw new IllegalArgumentException("bumper dimensions must be > 0");
    }
    this.bumperLengthXMeters = lengthXMeters;
    this.bumperWidthYMeters = widthYMeters;
    return this;
  }

  public DriveTrainSimulationConfig withTrackLengthTrackWidth(double trackLengthXMeters,
      double trackWidthYMeters) {
    if (!(trackLengthXMeters > 0.0) || !(trackWidthYMeters > 0.0)) {
      throw new IllegalArgumentException("track dimensions must be > 0");
    }
    this.trackLengthXMeters = trackLengthXMeters;
    this.trackWidthYMeters = trackWidthYMeters;
    return this;
  }

  public DriveTrainSimulationConfig withMaxWheelSpeedMps(double maxWheelSpeedMps) {
    if (!(maxWheelSpeedMps > 0.0)) {
      throw new IllegalArgumentException("maxWheelSpeedMps must be > 0");
    }
    this.maxWheelSpeedMps = maxWheelSpeedMps;
    return this;
  }

  public double robotMassKg() {
    return robotMassKg;
  }

  public double bumperLengthXMeters() {
    return bumperLengthXMeters;
  }

  public double bumperWidthYMeters() {
    return bumperWidthYMeters;
  }

  public double trackLengthXMeters() {
    return trackLengthXMeters;
  }

  public double trackWidthYMeters() {
    return trackWidthYMeters;
  }

  public double maxWheelSpeedMps() {
    return maxWheelSpeedMps;
  }
}