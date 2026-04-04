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
  /** Intake mounting side relative to robot heading. */
  public enum IntakeSide {
    FRONT,
    BACK,
    LEFT,
    RIGHT
  }

  private final Predicate<GamePieceOnFieldSimulation> filter;
  private final Queue<GamePieceOnFieldSimulation> obtained = new ArrayDeque<>();
  private boolean active = true;
  private double intakeWidthMeters = 0.8;
  private double intakeReachMeters = 0.5;
  private IntakeSide intakeSide = IntakeSide.FRONT;

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
   * Factory for in-the-frame intake style.
   */
  public static IntakeSimulation InTheFrameIntake(Predicate<GamePieceOnFieldSimulation> filter,
      IntakeSide side, double widthMeters, double reachMeters) {
    return inTheFrameIntake(filter, side, widthMeters, reachMeters);
  }

  /**
   * Repo-style lowerCamel alias for in-frame intake factory.
   */
  public static IntakeSimulation inTheFrameIntake(Predicate<GamePieceOnFieldSimulation> filter,
      IntakeSide side, double widthMeters, double reachMeters) {
    IntakeSimulation intake = new IntakeSimulation(filter);
    intake.intakeSide = Objects.requireNonNull(side);
    intake.intakeWidthMeters = Math.max(widthMeters, 0.01);
    intake.intakeReachMeters = Math.max(reachMeters, 0.01);
    return intake;
  }

  /**
   * Factory for over-the-bumper intake style.
   */
  public static IntakeSimulation OverTheBumperIntake(Predicate<GamePieceOnFieldSimulation> filter,
      IntakeSide side, double widthMeters, double reachMeters) {
    return overTheBumperIntake(filter, side, widthMeters, reachMeters);
  }

  /**
   * Repo-style lowerCamel alias for over-the-bumper intake factory.
   */
  public static IntakeSimulation overTheBumperIntake(Predicate<GamePieceOnFieldSimulation> filter,
      IntakeSide side, double widthMeters, double reachMeters) {
    IntakeSimulation intake = inTheFrameIntake(filter, side, widthMeters, reachMeters);
    intake.intakeReachMeters = Math.max(reachMeters * 1.2, 0.01);
    return intake;
  }

  /**
   * Activates the intake collector.
   */
  public void startIntake() {
    active = true;
  }

  /**
   * Deactivates the intake collector.
   */
  public void stopIntake() {
    active = false;
  }

  /**
   * Returns current intake activation state.
   */
  public boolean isActive() {
    return active;
  }

  /**
   * Scans and removes pieces within intake radius around robot pose.
   */
  public List<GamePieceOnFieldSimulation> removeObtainedGamePieces(SimulatedArena arena, Pose2 intakePose,
      double intakeRadiusMeters) {
    Objects.requireNonNull(arena);
    Objects.requireNonNull(intakePose);
    if (!active) {
      return List.of();
    }
    if (!(intakeRadiusMeters > 0.0)) {
      throw new IllegalArgumentException("intakeRadiusMeters must be > 0");
    }

    List<GamePieceOnFieldSimulation> removed = new ArrayList<>();
    Vec3 intakeCenter = intakeCenter(intakePose);
    for (GamePieceOnFieldSimulation piece : List.copyOf(arena.gamePiecesOnField())) {
      if (!filter.test(piece)) {
        continue;
      }
      double effectiveRadius = Math.max(intakeRadiusMeters, 0.5 * (intakeWidthMeters + intakeReachMeters));
      double distance = piece.pose().positionMeters().subtract(intakeCenter).norm();
      if (distance <= effectiveRadius) {
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

  private Vec3 intakeCenter(Pose2 intakePose) {
    double yaw = intakePose.yawRad();
    double frontX = Math.cos(yaw);
    double frontY = Math.sin(yaw);
    double leftX = -frontY;
    double leftY = frontX;

    double offsetX;
    double offsetY;
    double reachOffset = 0.5 * intakeReachMeters;
    double sideOffset = 0.5 * intakeWidthMeters;
    switch (intakeSide) {
      case FRONT -> {
        offsetX = frontX * reachOffset;
        offsetY = frontY * reachOffset;
      }
      case BACK -> {
        offsetX = -frontX * reachOffset;
        offsetY = -frontY * reachOffset;
      }
      case LEFT -> {
        offsetX = leftX * sideOffset;
        offsetY = leftY * sideOffset;
      }
      case RIGHT -> {
        offsetX = -leftX * sideOffset;
        offsetY = -leftY * sideOffset;
      }
      default -> {
        offsetX = 0.0;
        offsetY = 0.0;
      }
    }
    return new Vec3(intakePose.xMeters() + offsetX, intakePose.yMeters() + offsetY, 0.0);
  }
}