package rensim.simulation.seasonspecific.rebuilt2026;

import java.util.List;
import java.util.Random;
import rensim.Vec3;
import rensim.simulation.GamePieceOnFieldSimulation;
import rensim.simulation.GamePieceProjectile;
import rensim.simulation.Goal;
import rensim.simulation.Alliance;
import rensim.simulation.Pose2;
import rensim.simulation.SimulatedArena;
import rensim.simulation.SimulationOptions;

/**
 * 2026 Rebuilt arena with hub/outpost goals and configurable piece variance launch.
 */
public final class Arena2026Rebuilt extends SimulatedArena {
  public static final class RebuiltFieldObstacleMap extends FieldMap {
    public RebuiltFieldObstacleMap(boolean addRampCollider) {
      if (addRampCollider) {
        addRectangularObstacle(2.0, 0.6, new Pose2(8.27, 4.01, 0.0));
      }
    }
  }

  private static final String PIECE = "RebuiltPiece";
  private static final GamePieceOnFieldSimulation.GamePieceInfo PIECE_INFO =
      new GamePieceOnFieldSimulation.GamePieceInfo(PIECE, 0.3, 0.19, 0.05, 0.04, 0.4);

  private final Goal blueHub;
  private final Goal redHub;
  private final Goal blueOutpost;
  private final Goal redOutpost;
  private final Random random = new Random(2026);

  public Arena2026Rebuilt(boolean addRampCollider) {
    this(SimulationOptions.defaults(), addRampCollider);
  }

  public Arena2026Rebuilt(SimulationOptions options, boolean addRampCollider) {
    super(options, new RebuiltFieldObstacleMap(addRampCollider));
    double w = options.boundaries().widthMeters();
    blueHub = new Goal(this, PIECE, Alliance.BLUE, new Vec3(0.5, 3.2, 0.0), new Vec3(1.8, 4.8, 2.8));
    redHub = new Goal(this, PIECE, Alliance.RED, new Vec3(w - 1.8, 3.2, 0.0), new Vec3(w - 0.5, 4.8, 2.8));
    blueOutpost = new Goal(this, PIECE, Alliance.BLUE, new Vec3(3.4, 0.4, 0.0), new Vec3(4.6, 1.6, 2.0));
    redOutpost = new Goal(this, PIECE, Alliance.RED, new Vec3(w - 4.6, 0.4, 0.0), new Vec3(w - 3.4, 1.6, 2.0));
    addGoal(blueHub);
    addGoal(redHub);
    addGoal(blueOutpost);
    addGoal(redOutpost);
  }

  @Override
  public void placeGamePiecesOnField() {
    for (int i = 0; i < 8; i++) {
      addGamePiece(new GamePieceOnFieldSimulation(
          this,
          PIECE_INFO,
          new Pose2(2.0 + (i % 4) * 1.5, 2.0 + (i / 4) * 3.5, 0.0),
          Vec3.ZERO));
    }
  }

  /**
   * Spawns a projectile with randomized variance, similar to rebuilt outpost dump behavior.
   */
  public GamePieceProjectile addPieceWithVariance(Pose2 piecePose, double speedMps, double pitchRad,
      double xVariance, double yVariance, double yawVariance, double speedVariance,
      double pitchVariance) {
    double x = piecePose.xMeters() + randSym(xVariance);
    double y = piecePose.yMeters() + randSym(yVariance);
    double yaw = piecePose.yawRad() + randSym(yawVariance);
    double speed = Math.max(0.0, speedMps + randSym(speedVariance));
    double pitch = pitchRad + randSym(pitchVariance);

    Vec3 v = new Vec3(
        Math.cos(yaw) * Math.cos(pitch) * speed,
        Math.sin(yaw) * Math.cos(pitch) * speed,
        Math.sin(pitch) * speed);

    GamePieceProjectile projectile = new GamePieceProjectile(
        PIECE_INFO,
        new Vec3(x, y, 0.45),
        v,
        () -> new Vec3(options.boundaries().widthMeters() * 0.5, options.boundaries().heightMeters() * 0.5,
            1.0),
        new Vec3(0.7, 0.7, 0.7),
        options.gravityMps2().z(),
        () -> {}).enableBecomesGamePieceOnFieldAfterTouchGround();
    projectile.launch(options);
    addProjectile(projectile);
    return projectile;
  }

  /**
   * Manipulates all active pieces with a global velocity shift.
   */
  public void addGlobalPieceVelocity(Vec3 deltaVelocity) {
    for (GamePieceOnFieldSimulation piece : List.copyOf(gamePiecesOnField())) {
      applyGamePieceVelocityDelta(piece, deltaVelocity);
    }
  }

  public int scoreBlue() {
    return blueHub.scoredCount() + blueOutpost.scoredCount();
  }

  public int scoreRed() {
    return redHub.scoredCount() + redOutpost.scoredCount();
  }

  private double randSym(double magnitude) {
    return (random.nextDouble() * 2.0 - 1.0) * magnitude;
  }
}