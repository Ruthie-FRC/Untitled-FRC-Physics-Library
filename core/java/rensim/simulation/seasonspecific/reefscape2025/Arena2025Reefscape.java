package rensim.simulation.seasonspecific.reefscape2025;

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
 * 2025 Reefscape arena with coral and algae piece simulation.
 */
public final class Arena2025Reefscape extends SimulatedArena {
  public static final class ReefscapeFieldObstacleMap extends FieldMap {
    public ReefscapeFieldObstacleMap() {
      addRectangularObstacle(1.8, 1.8, new Pose2(4.5, 4.0, 0.0));
      addRectangularObstacle(1.8, 1.8, new Pose2(12.0, 4.0, 0.0));
    }
  }

  private static final String CORAL = "Coral";
  private static final String ALGAE = "Algae";
  private static final GamePieceOnFieldSimulation.GamePieceInfo CORAL_INFO =
      new GamePieceOnFieldSimulation.GamePieceInfo(CORAL, 0.28, 0.17, 0.05, 0.04, 0.42);
  private static final GamePieceOnFieldSimulation.GamePieceInfo ALGAE_INFO =
      new GamePieceOnFieldSimulation.GamePieceInfo(ALGAE, 0.32, 0.20, 0.05, 0.04, 0.38);

  private final Goal blueReef;
  private final Goal redReef;
  private final Random random = new Random(2025);

  public Arena2025Reefscape() {
    this(SimulationOptions.defaults());
  }

  public Arena2025Reefscape(SimulationOptions options) {
    super(options, new ReefscapeFieldObstacleMap());
    blueReef = new Goal(this, CORAL, Alliance.BLUE, new Vec3(3.3, 3.0, 0.0), new Vec3(5.7, 5.0, 2.5));
    redReef = new Goal(this, CORAL, Alliance.RED, new Vec3(10.9, 3.0, 0.0), new Vec3(13.2, 5.0, 2.5));
    addGoal(blueReef);
    addGoal(redReef);
  }

  @Override
  public void placeGamePiecesOnField() {
    for (int i = 0; i < 6; i++) {
      addGamePiece(new GamePieceOnFieldSimulation(this, CORAL_INFO,
          new Pose2(2.2 + i * 2.1, 1.2 + (i % 2) * 5.6, 0.0), Vec3.ZERO));
      addGamePiece(new GamePieceOnFieldSimulation(this, ALGAE_INFO,
          new Pose2(2.8 + i * 2.0, 2.2 + (i % 2) * 3.4, 0.0), Vec3.ZERO));
    }
  }

  /**
   * Launches coral projectile with optional randomization.
   */
  public GamePieceProjectile launchCoral(Pose2 fromPose, double speedMps, double verticalMps,
      boolean noisy) {
    double jitter = noisy ? (random.nextDouble() - 0.5) * 0.3 : 0.0;
    Vec3 launchVel = new Vec3(
        Math.cos(fromPose.yawRad() + jitter) * speedMps,
        Math.sin(fromPose.yawRad() + jitter) * speedMps,
        verticalMps + jitter * 0.5);
    GamePieceProjectile projectile = new GamePieceProjectile(
        CORAL_INFO,
        new Vec3(fromPose.xMeters(), fromPose.yMeters(), 0.45),
        launchVel,
        () -> new Vec3(4.5, 4.0, 1.0),
        new Vec3(0.6, 0.6, 0.6),
        options.gravityMps2().z(),
        () -> {}).enableBecomesGamePieceOnFieldAfterTouchGround();
    projectile.launch(options);
    addProjectile(projectile);
    return projectile;
  }

  /**
   * Manipulates algae by pushing them away from a center point.
   */
  public void repelAlgae(Vec3 center, double speedBoostMps) {
    for (GamePieceOnFieldSimulation piece : List.copyOf(gamePiecesOnField())) {
      if (!piece.type().equals(ALGAE)) {
        continue;
      }
      Vec3 delta = piece.pose().positionMeters().subtract(center);
      Vec3 dir = delta.norm() < 1.0e-6 ? new Vec3(1.0, 0.0, 0.0) : delta.normalized();
      applyGamePieceVelocityDelta(piece, dir.scale(speedBoostMps));
    }
  }

  public int reefScoreBlue() {
    return blueReef.scoredCount();
  }

  public int reefScoreRed() {
    return redReef.scoredCount();
  }
}