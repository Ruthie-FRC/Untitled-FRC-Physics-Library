package rensim.simulation.manipulation;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.function.Supplier;
import rensim.Vec3;
import rensim.simulation.GamePieceOnFieldSimulation;
import rensim.simulation.GamePieceProjectile;
import rensim.simulation.IntakeSimulation;
import rensim.simulation.Pose2;
import rensim.simulation.SimulatedArena;
import rensim.simulation.SimulationOptions;
import rensim.simulation.gamepiece.GamePieceData;
import rensim.simulation.projectile.ProjectileLaunchPlan;
import rensim.simulation.projectile.ProjectileTargetingSolver;

/**
 * End-effector style helper that intakes, stores, and launches game pieces.
 */
public final class GamePieceManipulator {
  private final SimulatedArena arena;
  private final IntakeSimulation intake;
  private final Deque<GamePieceOnFieldSimulation.GamePieceInfo> heldPieces = new ArrayDeque<>();
  private final int capacity;

  public GamePieceManipulator(SimulatedArena arena, IntakeSimulation intake, int capacity) {
    this.arena = Objects.requireNonNull(arena);
    this.intake = Objects.requireNonNull(intake);
    if (capacity < 1) {
      throw new IllegalArgumentException("capacity must be >= 1");
    }
    this.capacity = capacity;
  }

  public int heldCount() {
    return heldPieces.size();
  }

  public boolean isFull() {
    return heldPieces.size() >= capacity;
  }

  /**
   * Runs intake scan and stores pieces up to manipulator capacity.
   */
  public int intakeNearby(Pose2 robotPose, double intakeRadiusMeters) {
    List<GamePieceOnFieldSimulation> obtained = intake.removeObtainedGamePieces(arena, robotPose, intakeRadiusMeters);
    int accepted = 0;
    for (GamePieceOnFieldSimulation piece : obtained) {
      if (isFull()) {
        break;
      }
      heldPieces.addLast(piece.info());
      accepted++;
    }
    return accepted;
  }

  /**
   * Loads a virtual piece into storage.
   */
  public boolean preload(GamePieceData data) {
    Objects.requireNonNull(data);
    if (isFull()) {
      return false;
    }
    heldPieces.addLast(data.toInfo());
    return true;
  }

  /**
   * Ejects one held piece back onto the field.
   */
  public Optional<GamePieceOnFieldSimulation> ejectToField(Pose2 pose, Vec3 velocityMps) {
    Objects.requireNonNull(pose);
    Objects.requireNonNull(velocityMps);
    if (heldPieces.isEmpty()) {
      return Optional.empty();
    }
    GamePieceOnFieldSimulation piece = new GamePieceOnFieldSimulation(
        arena,
        heldPieces.removeFirst(),
        pose,
        velocityMps);
    arena.addGamePiece(piece);
    return Optional.of(piece);
  }

  /**
   * Launches one held piece toward a target with ballistic speed solver.
   */
  public Optional<GamePieceProjectile> launchAtTarget(Pose2 launchPose, double launchHeightMeters,
      Vec3 target, double launchSpeedMps, Supplier<Vec3> dynamicTargetSupplier, Runnable onHit) {
    Objects.requireNonNull(launchPose);
    Objects.requireNonNull(target);
    Objects.requireNonNull(dynamicTargetSupplier);
    Objects.requireNonNull(onHit);
    if (heldPieces.isEmpty()) {
      return Optional.empty();
    }

    Vec3 from = new Vec3(launchPose.xMeters(), launchPose.yMeters(), launchHeightMeters);
    SimulationOptions options = arena.simulationOptions();
    Optional<ProjectileLaunchPlan> plan = ProjectileTargetingSolver.solveBySpeed(
        from,
        target,
        launchSpeedMps,
        options.gravityMps2().z(),
        false);
    if (plan.isEmpty()) {
      return Optional.empty();
    }

    GamePieceOnFieldSimulation.GamePieceInfo info = heldPieces.removeFirst();
    GamePieceProjectile projectile = new GamePieceProjectile(
        info,
        from,
        plan.get().velocityMps(),
        dynamicTargetSupplier,
        new Vec3(0.5, 0.5, 0.5),
        options.gravityMps2().z(),
        onHit);
    projectile.enableBecomesGamePieceOnFieldAfterTouchGround();
    projectile.launch(options);
    arena.addProjectile(projectile);
    return Optional.of(projectile);
  }
}
