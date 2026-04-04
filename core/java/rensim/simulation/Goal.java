package rensim.simulation;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.Predicate;
import rensim.Vec3;

/**
 * Generic scoring goal that consumes game pieces entering a configured volume.
 */
public class Goal implements SimulatedArena.Simulatable {
  private final SimulatedArena arena;
  private final String acceptedType;
  private final Alliance ownerAlliance;
  private final Vec3 min;
  private final Vec3 max;
  private final Predicate<GamePieceOnFieldSimulation> velocityValidator;
  private int scoredCount;

  /**
   * Creates a goal with axis-aligned bounds and piece type filter.
   */
  public Goal(SimulatedArena arena, String acceptedType, Vec3 min, Vec3 max,
      Predicate<GamePieceOnFieldSimulation> velocityValidator) {
    this(arena, acceptedType, Alliance.NEUTRAL, min, max, velocityValidator);
  }

  /**
   * Creates a goal with explicit alliance ownership.
   */
  public Goal(SimulatedArena arena, String acceptedType, Alliance ownerAlliance, Vec3 min, Vec3 max,
      Predicate<GamePieceOnFieldSimulation> velocityValidator) {
    this.arena = Objects.requireNonNull(arena);
    this.acceptedType = Objects.requireNonNull(acceptedType);
    this.ownerAlliance = Objects.requireNonNull(ownerAlliance);
    this.min = Objects.requireNonNull(min);
    this.max = Objects.requireNonNull(max);
    this.velocityValidator = Objects.requireNonNull(velocityValidator);
  }

  /**
   * Convenience goal accepting all entry velocities.
   */
  public Goal(SimulatedArena arena, String acceptedType, Vec3 min, Vec3 max) {
    this(arena, acceptedType, Alliance.NEUTRAL, min, max, piece -> true);
  }

  /**
   * Convenience goal with explicit alliance and accepting all entry velocities.
   */
  public Goal(SimulatedArena arena, String acceptedType, Alliance ownerAlliance, Vec3 min, Vec3 max) {
    this(arena, acceptedType, ownerAlliance, min, max, piece -> true);
  }

  @Override
  public void simulationSubTick(int subTickNum) {
    List<GamePieceOnFieldSimulation> toScore = new ArrayList<>();
    for (GamePieceOnFieldSimulation piece : arena.gamePiecesOnField()) {
      if (!piece.type().equals(acceptedType)) {
        continue;
      }
      if (!velocityValidator.test(piece)) {
        continue;
      }
      Vec3 p = piece.pose().positionMeters();
      if (contains(p)) {
        toScore.add(piece);
      }
    }

    for (GamePieceOnFieldSimulation piece : toScore) {
      arena.removePiece(piece);
      piece.triggerHitTargetCallback();
      scoredCount++;
    }
  }

  /**
   * Returns total pieces scored in this goal.
   */
  public int scoredCount() {
    return scoredCount;
  }

  /**
   * Returns owning alliance for this goal.
   */
  public Alliance ownerAlliance() {
    return ownerAlliance;
  }

  private boolean contains(Vec3 p) {
    return p.x() >= min.x() && p.x() <= max.x()
        && p.y() >= min.y() && p.y() <= max.y()
        && p.z() >= min.z() && p.z() <= max.z();
  }
}