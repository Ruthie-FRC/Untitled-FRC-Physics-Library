package rensim.simulation;

import java.util.Objects;
import rensim.PhysicsBody;
import rensim.Vec3;

/**
 * Grounded game piece backed by a rigid body in {@link SimulatedArena}.
 */
public class GamePieceOnFieldSimulation implements GamePiece {
  /**
   * Metadata used to define simulated game-piece behavior.
   */
  public record GamePieceInfo(String type, double massKg, double radiusMeters, double linearDamping,
      double angularDamping, double coefficientOfRestitution) {
    public GamePieceInfo {
      if (type == null || type.isBlank()) {
        throw new IllegalArgumentException("type cannot be blank");
      }
      if (!(massKg > 0.0)) {
        throw new IllegalArgumentException("massKg must be > 0");
      }
      if (!(radiusMeters > 0.0)) {
        throw new IllegalArgumentException("radiusMeters must be > 0");
      }
      if (linearDamping < 0.0 || angularDamping < 0.0) {
        throw new IllegalArgumentException("damping must be >= 0");
      }
      if (coefficientOfRestitution < 0.0 || coefficientOfRestitution > 1.0) {
        throw new IllegalArgumentException("coefficientOfRestitution must be in [0, 1]");
      }
    }
  }

  private final SimulatedArena arena;
  private final PhysicsBody body;
  private final GamePieceInfo info;
  private double yawRad;

  /**
   * Creates a grounded piece at an initial field pose.
   */
  public GamePieceOnFieldSimulation(SimulatedArena arena, GamePieceInfo info, Pose2 initialPose,
      Vec3 initialVelocityMps) {
    this.arena = Objects.requireNonNull(arena);
    this.info = Objects.requireNonNull(info);
    Objects.requireNonNull(initialPose);
    Objects.requireNonNull(initialVelocityMps);

    this.body = arena.world().createBody(info.massKg());
    this.body.setPosition(new Vec3(initialPose.xMeters(), initialPose.yMeters(), 0.0));
    this.body.setLinearVelocity(initialVelocityMps);
    this.body.setGravityEnabled(false);
    this.body.setSphereCollider(info.radiusMeters(), info.coefficientOfRestitution());
    this.yawRad = initialPose.yawRad();
  }

  /**
   * Sets world-space velocity for this piece.
   */
  public void setVelocity(Vec3 worldVelocityMps) {
    body.setLinearVelocity(worldVelocityMps);
  }

  /**
   * Gets 2D pose on field plane.
   */
  public Pose2 poseOnField() {
    Vec3 p = body.position();
    return new Pose2(p.x(), p.y(), yawRad);
  }

  @Override
  public String type() {
    return info.type();
  }

  @Override
  public Pose3 pose() {
    Vec3 p = body.position();
    return new Pose3(new Vec3(p.x(), p.y(), 0.0), yawRad);
  }

  @Override
  public Vec3 velocityMps() {
    return body.linearVelocity();
  }

  @Override
  public boolean grounded() {
    return true;
  }

  @Override
  public void triggerHitTargetCallback() {
    // intentionally no-op by default
  }

  PhysicsBody body() {
    return body;
  }

  SimulatedArena arena() {
    return arena;
  }
}