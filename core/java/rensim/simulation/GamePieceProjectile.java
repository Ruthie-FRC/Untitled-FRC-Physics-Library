package rensim.simulation;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.Consumer;
import java.util.function.Supplier;
import rensim.Vec3;

/**
 * Ballistic game piece with optional trajectory preview and ground conversion.
 */
public class GamePieceProjectile implements GamePiece {
  private final GamePieceOnFieldSimulation.GamePieceInfo info;
  private final Vec3 initialPositionMeters;
  private final Vec3 initialVelocityMps;
  private final Supplier<Vec3> targetPositionSupplier;
  private final Vec3 targetToleranceMeters;
  private final double gravityMps2;
  private final Runnable hitTargetCallback;

  private Consumer<List<Pose3>> trajectoryPreviewConsumer = points -> {};
  private boolean becomeGroundedAfterTouchGround = false;
  private double touchGroundHeightMeters = 0.05;

  private boolean launched;
  private boolean hitTarget;
  private boolean hitGround;
  private boolean outOfField;
  private double timeSinceLaunchSeconds;

  /**
   * Creates a projectile simulation with target checking.
   */
  public GamePieceProjectile(GamePieceOnFieldSimulation.GamePieceInfo info, Vec3 initialPositionMeters,
      Vec3 initialVelocityMps, Supplier<Vec3> targetPositionSupplier, Vec3 targetToleranceMeters,
      double gravityMps2, Runnable hitTargetCallback) {
    this.info = Objects.requireNonNull(info);
    this.initialPositionMeters = Objects.requireNonNull(initialPositionMeters);
    this.initialVelocityMps = Objects.requireNonNull(initialVelocityMps);
    this.targetPositionSupplier = Objects.requireNonNull(targetPositionSupplier);
    this.targetToleranceMeters = Objects.requireNonNull(targetToleranceMeters);
    this.gravityMps2 = gravityMps2;
    this.hitTargetCallback = Objects.requireNonNull(hitTargetCallback);
  }

  /**
   * Enables replacement by grounded game piece on touchdown.
   */
  public GamePieceProjectile enableBecomesGamePieceOnFieldAfterTouchGround() {
    this.becomeGroundedAfterTouchGround = true;
    return this;
  }

  /**
   * Disables replacement by grounded game piece on touchdown.
   */
  public GamePieceProjectile disableBecomesGamePieceOnFieldAfterTouchGround() {
    this.becomeGroundedAfterTouchGround = false;
    return this;
  }

  /**
   * Sets trajectory preview callback.
   */
  public GamePieceProjectile withProjectileTrajectoryDisplayCallBack(
      Consumer<List<Pose3>> trajectoryPreviewConsumer) {
    this.trajectoryPreviewConsumer = Objects.requireNonNull(trajectoryPreviewConsumer);
    return this;
  }

  /**
   * Starts projectile simulation and computes trajectory preview.
   */
  public void launch(SimulationOptions options) {
    Objects.requireNonNull(options);
    this.launched = true;
    this.hitTarget = false;
    this.hitGround = false;
    this.outOfField = false;
    this.timeSinceLaunchSeconds = 0.0;

    int previewSteps = options.projectile().previewSteps();
    double step = options.timing().fixedDtSeconds() * options.timing().subTicksPerRobotPeriod();
    List<Pose3> preview = new ArrayList<>(previewSteps);
    for (int i = 0; i < previewSteps; i++) {
      double t = i * step;
      Pose3 pose = poseAt(t);
      preview.add(pose);
      if (pose.positionMeters().z() <= touchGroundHeightMeters && t * Math.abs(gravityMps2)
          > Math.max(initialVelocityMps.z(), 0.0)) {
        break;
      }
    }
    trajectoryPreviewConsumer.accept(List.copyOf(preview));
  }

  /**
   * Advances projectile by one time step.
   */
  public void update(double dtSeconds, SimulationOptions options) {
    if (!launched || hitGround || outOfField || hitTarget) {
      return;
    }
    timeSinceLaunchSeconds += dtSeconds;

    Vec3 current = positionAt(timeSinceLaunchSeconds);
    Vec3 target = targetPositionSupplier.get();
    Vec3 delta = target.subtract(current);
    if (Math.abs(delta.x()) <= targetToleranceMeters.x()
        && Math.abs(delta.y()) <= targetToleranceMeters.y()
        && Math.abs(delta.z()) <= targetToleranceMeters.z()) {
      hitTarget = true;
      hitTargetCallback.run();
      return;
    }

    if (current.z() <= touchGroundHeightMeters) {
      hitGround = true;
      return;
    }

    double edge = 2.0;
    if (current.x() < -edge || current.x() > options.boundaries().widthMeters() + edge
        || current.y() < -edge || current.y() > options.boundaries().heightMeters() + edge) {
      outOfField = true;
    }
  }

  /**
   * Converts projectile into grounded piece when touchdown mode is enabled.
   */
  public GamePieceOnFieldSimulation addGamePieceAfterTouchGround(SimulatedArena arena) {
    if (!becomeGroundedAfterTouchGround || !hitGround) {
      return null;
    }
    Vec3 p = positionAt(timeSinceLaunchSeconds);
    return new GamePieceOnFieldSimulation(
        arena,
        info,
        new Pose2(p.x(), p.y(), 0.0),
        new Vec3(initialVelocityMps.x(), initialVelocityMps.y(), 0.0));
  }

  /**
   * Returns true when projectile hit ground.
   */
  public boolean hasHitGround() {
    return hitGround;
  }

  /**
   * Returns true when projectile hit target.
   */
  public boolean hasHitTarget() {
    return hitTarget;
  }

  /**
   * Returns true when projectile left field bounds.
   */
  public boolean hasGoneOutOfField() {
    return outOfField;
  }

  /**
   * Marks projectile as cleaned up and returns itself for fluent use.
   */
  public GamePieceProjectile cleanUp() {
    this.outOfField = true;
    return this;
  }

  @Override
  public String type() {
    return info.type();
  }

  @Override
  public Pose3 pose() {
    return poseAt(timeSinceLaunchSeconds);
  }

  @Override
  public Vec3 velocityMps() {
    return velocityAt(timeSinceLaunchSeconds);
  }

  @Override
  public boolean grounded() {
    return false;
  }

  @Override
  public void triggerHitTargetCallback() {
    hitTargetCallback.run();
  }

  private Pose3 poseAt(double t) {
    return new Pose3(positionAt(t), 0.0);
  }

  private Vec3 positionAt(double t) {
    double z = initialPositionMeters.z() + (initialVelocityMps.z() * t) + (0.5 * gravityMps2 * t * t);
    return new Vec3(
        initialPositionMeters.x() + initialVelocityMps.x() * t,
        initialPositionMeters.y() + initialVelocityMps.y() * t,
        z);
  }

  private Vec3 velocityAt(double t) {
    return new Vec3(initialVelocityMps.x(), initialVelocityMps.y(), initialVelocityMps.z() + gravityMps2 * t);
  }
}