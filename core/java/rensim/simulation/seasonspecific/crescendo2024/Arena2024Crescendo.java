package rensim.simulation.seasonspecific.crescendo2024;

import java.util.List;
import rensim.Vec3;
import rensim.simulation.GamePieceOnFieldSimulation;
import rensim.simulation.GamePieceProjectile;
import rensim.simulation.Goal;
import rensim.simulation.Alliance;
import rensim.simulation.Pose2;
import rensim.simulation.SimulatedArena;
import rensim.simulation.SimulationOptions;

/**
 * 2024 Crescendo arena with note pieces, speaker/amp scoring and launch mechanics.
 */
public final class Arena2024Crescendo extends SimulatedArena {
  public static final class CrescendoFieldObstaclesMap extends FieldMap {
    public CrescendoFieldObstaclesMap() {
      addRectangularObstacle(0.4, 1.7, new Pose2(8.27, 4.01, 0.0));
    }
  }

  private static final String NOTE = "Note";
  private static final GamePieceOnFieldSimulation.GamePieceInfo NOTE_INFO =
      new GamePieceOnFieldSimulation.GamePieceInfo(NOTE, 0.24, 0.18, 0.04, 0.03, 0.35);

  private final Goal blueSpeaker;
  private final Goal redSpeaker;
  private final Goal blueAmp;
  private final Goal redAmp;

  public Arena2024Crescendo() {
    this(SimulationOptions.defaults());
  }

  public Arena2024Crescendo(SimulationOptions options) {
    super(options, new CrescendoFieldObstaclesMap());
    blueSpeaker = new Goal(this, NOTE, Alliance.BLUE, new Vec3(0.0, 4.7, 0.0), new Vec3(1.0, 6.2, 3.0));
    redSpeaker = new Goal(this, NOTE, Alliance.RED,
        new Vec3(options.boundaries().widthMeters() - 1.0, 4.7, 0.0),
        new Vec3(options.boundaries().widthMeters(), 6.2, 3.0));
    blueAmp = new Goal(this, NOTE, Alliance.BLUE, new Vec3(1.6, 7.0, 0.0), new Vec3(2.5, 8.02, 2.0));
    redAmp = new Goal(this, NOTE, Alliance.RED,
        new Vec3(options.boundaries().widthMeters() - 2.5, 7.0, 0.0),
        new Vec3(options.boundaries().widthMeters() - 1.6, 8.02, 2.0));
    addGoal(blueSpeaker);
    addGoal(redSpeaker);
    addGoal(blueAmp);
    addGoal(redAmp);
  }

  @Override
  public void placeGamePiecesOnField() {
    double[][] starts = {
        {8.27, 1.12},
        {8.27, 2.44},
        {8.27, 4.01},
        {8.27, 5.56},
        {8.27, 6.88}
    };
    for (double[] s : starts) {
      addGamePiece(new GamePieceOnFieldSimulation(this, NOTE_INFO, new Pose2(s[0], s[1], 0.0), Vec3.ZERO));
    }
  }

  /**
   * Launches a note projectile from shooter pose.
   */
  public GamePieceProjectile launchNote(Pose2 shooterPose, double speedMps, double verticalSpeedMps,
      Vec3 target, Runnable onHit) {
    Vec3 horizontal = new Vec3(Math.cos(shooterPose.yawRad()) * speedMps,
        Math.sin(shooterPose.yawRad()) * speedMps,
        verticalSpeedMps);
    GamePieceProjectile projectile = new GamePieceProjectile(
        NOTE_INFO,
        new Vec3(shooterPose.xMeters(), shooterPose.yMeters(), 0.4),
        horizontal,
        () -> target,
        new Vec3(0.45, 0.45, 0.45),
        options.gravityMps2().z(),
        onHit).enableBecomesGamePieceOnFieldAfterTouchGround();
    projectile.launch(options);
    addProjectile(projectile);
    return projectile;
  }

  /**
   * Applies random manipulation burst to all notes in play.
   */
  public void scrambleGroundNotes(double vx, double vy) {
    for (GamePieceOnFieldSimulation piece : List.copyOf(gamePiecesOnField())) {
      if (piece.type().equals(NOTE)) {
        applyGamePieceVelocityDelta(piece, new Vec3(vx, vy, 0.0));
      }
    }
  }

  public int blueScore() {
    return blueSpeaker.scoredCount() + blueAmp.scoredCount();
  }

  public int redScore() {
    return redSpeaker.scoredCount() + redAmp.scoredCount();
  }
}