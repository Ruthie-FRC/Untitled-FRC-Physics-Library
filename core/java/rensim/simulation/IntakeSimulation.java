package rensim.simulation;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Queue;
import java.util.function.Predicate;
import rensim.Vec3;

/**
 * Simple intake simulator that collects nearby grounded game pieces.
 */
public final class IntakeSimulation {
  private final Predicate<GamePieceOnFieldSimulation> filter;
  private final Queue<GamePieceOnFieldSimulation> obtained = new ArrayDeque<>();

  /**
   * Creates an intake simulation with custom filtering logic.
   */
  public IntakeSimulation(Predicate<GamePieceOnFieldSimulation> filter) {
    this.filter = Objects.requireNonNull(filter);
  }

  /**
   * Default intake simulation that accepts all grounded pieces.
   */
  public static IntakeSimulation acceptingAll() {
    return new IntakeSimulation(piece -> true);
  }

  /**
   * Scans and removes pieces within intake radius around robot pose.
   */
  public List<GamePieceOnFieldSimulation> removeObtainedGamePieces(SimulatedArena arena, Pose2 intakePose,
      double intakeRadiusMeters) {
    Objects.requireNonNull(arena);
    Objects.requireNonNull(intakePose);
    if (!(intakeRadiusMeters > 0.0)) {
      throw new IllegalArgumentException("intakeRadiusMeters must be > 0");
    }

    List<GamePieceOnFieldSimulation> removed = new ArrayList<>();
    Vec3 intakeCenter = new Vec3(intakePose.xMeters(), intakePose.yMeters(), 0.0);
    for (GamePieceOnFieldSimulation piece : List.copyOf(arena.gamePiecesOnField())) {
      if (!filter.test(piece)) {
        continue;
      }
      double distance = piece.pose().positionMeters().subtract(intakeCenter).norm();
      if (distance <= intakeRadiusMeters) {
        arena.removePiece(piece);
        obtained.offer(piece);
        removed.add(piece);
      }
    }
    return removed;
  }

  /**
   * Returns and clears pieces obtained since the last call.
   */
  public List<GamePieceOnFieldSimulation> drainObtainedPieces() {
    List<GamePieceOnFieldSimulation> drained = new ArrayList<>();
    while (!obtained.isEmpty()) {
      drained.add(obtained.poll());
    }
    return drained;
  }
}