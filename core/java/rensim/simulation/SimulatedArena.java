package rensim.simulation;

import frcsim_physics.RigidBody;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import rensim.PhysicsBody;
import rensim.PhysicsWorld;
import rensim.Vec3;
import rensim.simulation.drivesims.SwerveDriveSimulation;
import rensim.simulation.gamepiece.GamePieceData;
import rensim.simulation.projectile.ProjectileLaunchPlan;
import rensim.simulation.projectile.ProjectileTargetingSolver;
import rensim.simulation.seasonspecific.crescendo2024.Arena2024Crescendo;

/**
 * Maple-style simulation world orchestrator using RenSim physics primitives.
 */
public abstract class SimulatedArena {
  private static int simulationSubTicksInOnePeriod = 5;
  private static double robotPeriodSeconds = 0.02;
  private static SimulatedArena instance = new Arena2024Crescendo();

  /**
   * Returns global simulation arena instance.
   */
  public static synchronized SimulatedArena getInstance() {
    return instance;
  }

  /**
   * Replaces global simulation arena instance.
   */
  public static synchronized void overrideInstance(SimulatedArena newInstance) {
    instance = Objects.requireNonNull(newInstance);
  }

  /**
   * Overrides global simulation timing settings.
   */
  public static synchronized void overrideSimulationTimings(double robotPeriodSeconds,
      int simulationSubTicksInOnePeriod) {
    if (!(robotPeriodSeconds > 0.0)) {
      throw new IllegalArgumentException("robotPeriodSeconds must be > 0");
    }
    if (simulationSubTicksInOnePeriod < 1) {
      throw new IllegalArgumentException("simulationSubTicksInOnePeriod must be >= 1");
    }
    SimulatedArena.robotPeriodSeconds = robotPeriodSeconds;
    SimulatedArena.simulationSubTicksInOnePeriod = simulationSubTicksInOnePeriod;
  }

  /**
   * Gets configured simulation sub-ticks in one robot period.
   */
  public static synchronized int getSimulationSubTicksInOnePeriod() {
    return simulationSubTicksInOnePeriod;
  }

  /**
   * Gets configured robot period in seconds.
   */
  public static synchronized double getRobotPeriodSeconds() {
    return robotPeriodSeconds;
  }

  /**
   * Custom simulation callback run each sub-tick.
   */
  @FunctionalInterface
  public interface Simulatable {
    void simulationSubTick(int subTickNum);
  }

  /**
   * Field map abstraction for boundary and obstacle setup.
   */
  public abstract static class FieldMap {
    private final List<LineSegment> borderLines = new ArrayList<>();
    private final List<RectObstacle> rectangularObstacles = new ArrayList<>();

    protected void addBorderLine(double startX, double startY, double endX, double endY) {
      borderLines.add(new LineSegment(startX, startY, endX, endY));
    }

    protected void addRectangularObstacle(double widthMeters, double heightMeters, Pose2 pose) {
      if (!(widthMeters > 0.0) || !(heightMeters > 0.0)) {
        throw new IllegalArgumentException("Obstacle dimensions must be > 0");
      }
      rectangularObstacles.add(new RectObstacle(widthMeters, heightMeters, pose));
    }

    List<LineSegment> borderLines() {
      return List.copyOf(borderLines);
    }

    List<RectObstacle> rectangularObstacles() {
      return List.copyOf(rectangularObstacles);
    }
  }

  /**
   * 2D line segment used for border storage.
   */
  public record LineSegment(double startX, double startY, double endX, double endY) {}

  /**
   * Axis-aligned rectangle obstacle descriptor.
   */
  public record RectObstacle(double widthMeters, double heightMeters, Pose2 pose) {}

  private record RoundBody(PhysicsBody body, double radiusMeters) {}

  protected final SimulationOptions options;
  protected final FieldMap fieldMap;
  protected final PhysicsWorld world;
  protected final Map<String, Double> blueMatchBreakdown = new LinkedHashMap<>();
  protected final Map<String, Double> redMatchBreakdown = new LinkedHashMap<>();

  protected final List<SwerveDriveSimulation> driveTrainSimulations = new ArrayList<>();
  protected final List<Simulatable> customSimulations = new ArrayList<>();
  protected final List<Goal> goals = new ArrayList<>();
  protected final List<IntakeSimulation> intakeSimulations = new ArrayList<>();
  protected final Set<GamePieceOnFieldSimulation> gamePiecesOnField = new HashSet<>();
  protected final Set<GamePieceProjectile> gamePieceProjectiles = new HashSet<>();
  private final List<RoundBody> roundBodies = new ArrayList<>();
  private double matchClockSeconds = 135.0;
  private boolean publishMatchBreakdown;

  /**
   * Creates a simulated arena with configurable options and field map.
   */
  protected SimulatedArena(SimulationOptions options, FieldMap fieldMap) {
    this.options = Objects.requireNonNull(options);
    this.fieldMap = Objects.requireNonNull(fieldMap);
    this.world = new PhysicsWorld(options.timing().fixedDtSeconds(), true);
    this.world.setGravity(options.gravityMps2());
    this.world.setSimpleSphereCollisionsEnabled(options.collision().enabled());
  }

  /**
   * Steps full robot-period simulation using configured sub-ticks.
   */
  public synchronized void simulationPeriodic() {
    for (int subTick = 0; subTick < options.timing().subTicksPerRobotPeriod(); subTick++) {
      simulationSubTick(subTick);
    }

    matchClockSeconds = Math.max(0.0, matchClockSeconds - getRobotPeriodSeconds());
  }

  /**
   * Runs one sub-tick update pass.
   */
  protected void simulationSubTick(int subTickNum) {
    for (SwerveDriveSimulation drive : driveTrainSimulations) {
      drive.simulationSubTick(options.timing().fixedDtSeconds());
    }

    for (GamePieceProjectile projectile : List.copyOf(gamePieceProjectiles)) {
      projectile.update(options.timing().fixedDtSeconds(), options);
      if (projectile.hasHitGround()) {
        GamePieceOnFieldSimulation grounded = projectile.addGamePieceAfterTouchGround(this);
        if (grounded != null) {
          addGamePiece(grounded);
        }
      }
      if (projectile.hasHitGround() || projectile.hasHitTarget() || projectile.hasGoneOutOfField()) {
        removePiece(projectile);
      }
    }

    world.step();

    if (options.boundaries().enableWalls()) {
      applyBoundaryConstraints();
    }

    for (IntakeSimulation intake : intakeSimulations) {
      intake.removeObtainedGamePieces(this, defaultIntakePose(), options.tolerances().intakeRadiusMeters());
    }

    for (Simulatable custom : customSimulations) {
      custom.simulationSubTick(subTickNum);
    }

    for (Goal goal : goals) {
      goal.simulationSubTick(subTickNum);
    }

    replaceValueInMatchBreakdown(true, "TotalScore", totalScoreBlue());
    replaceValueInMatchBreakdown(false, "TotalScore", totalScoreRed());
  }

  /**
   * Override to define autonomous-period game-piece placements.
   */
  public abstract void placeGamePiecesOnField();

  /**
   * Registers drivetrain simulation.
   */
  public synchronized void addDriveTrainSimulation(SwerveDriveSimulation driveTrainSimulation) {
    driveTrainSimulations.add(Objects.requireNonNull(driveTrainSimulation));
  }

  /**
   * Registers opponent robot simulation behavior.
   */
  public synchronized void addOpponentRobotSimulation(OpponentRobotSimulation opponentSimulation) {
    Objects.requireNonNull(opponentSimulation);
    addDriveTrainSimulation(opponentSimulation.drive());
    addCustomSimulation(opponentSimulation);
  }

  /**
   * Registers intake simulation.
   */
  public synchronized void addIntakeSimulation(IntakeSimulation intakeSimulation) {
    intakeSimulations.add(Objects.requireNonNull(intakeSimulation));
  }

  /**
   * Registers custom simulation callback.
   */
  public synchronized void addCustomSimulation(Simulatable customSimulation) {
    customSimulations.add(Objects.requireNonNull(customSimulation));
  }

  /**
   * Registers a scoring goal simulation.
   */
  public synchronized void addGoal(Goal goal) {
    goals.add(Objects.requireNonNull(goal));
  }

  /**
   * Adds grounded game piece.
   */
  public synchronized void addGamePiece(GamePieceOnFieldSimulation piece) {
    gamePiecesOnField.add(Objects.requireNonNull(piece));
  }

  /**
   * Adds grounded game piece from canonical data definition.
   */
  public synchronized GamePieceOnFieldSimulation addGamePiece(GamePieceData data, Pose2 pose,
      Vec3 initialVelocityMps) {
    Objects.requireNonNull(data);
    Objects.requireNonNull(pose);
    Objects.requireNonNull(initialVelocityMps);
    GamePieceOnFieldSimulation piece =
        new GamePieceOnFieldSimulation(this, data.toInfo(), pose, initialVelocityMps);
    addGamePiece(piece);
    return piece;
  }

  /**
   * Adds projectile game piece.
   */
  public synchronized void addProjectile(GamePieceProjectile projectile) {
    gamePieceProjectiles.add(Objects.requireNonNull(projectile));
  }

  /**
   * Maple-style alias for projectile registration.
   */
  public synchronized void addGamePieceProjectile(GamePieceProjectile projectile) {
    addProjectile(projectile);
  }

  /**
   * Launches a projectile toward a target using fixed launch speed.
   */
  public synchronized GamePieceProjectile launchPieceAtTarget(GamePieceData data, Pose2 launchPose,
      double launchHeightMeters, Vec3 targetPositionMeters, double launchSpeedMps,
      boolean highArc, Runnable onHitTarget) {
    Objects.requireNonNull(data);
    Objects.requireNonNull(launchPose);
    Objects.requireNonNull(targetPositionMeters);
    Objects.requireNonNull(onHitTarget);

    Vec3 from = new Vec3(launchPose.xMeters(), launchPose.yMeters(), launchHeightMeters);
    ProjectileLaunchPlan plan = ProjectileTargetingSolver
        .solveBySpeed(from, targetPositionMeters, launchSpeedMps, options.gravityMps2().z(), highArc)
        .orElseThrow(() -> new IllegalArgumentException("No ballistic solution for given launch parameters"));

    GamePieceProjectile projectile = new GamePieceProjectile(
        data.toInfo(),
        from,
        plan.velocityMps(),
        () -> targetPositionMeters,
        new Vec3(0.5, 0.5, 0.5),
        options.gravityMps2().z(),
        onHitTarget).enableBecomesGamePieceOnFieldAfterTouchGround();
    projectile.launch(options);
    addProjectile(projectile);
    return projectile;
  }

  /**
   * Removes any game piece implementation from arena sets.
   */
  public synchronized void removePiece(GamePiece piece) {
    if (piece instanceof GamePieceOnFieldSimulation grounded) {
      gamePiecesOnField.remove(grounded);
    }
    if (piece instanceof GamePieceProjectile projectile) {
      gamePieceProjectiles.remove(projectile);
    }
  }

  /**
   * Returns all grounded pieces on field.
   */
  public synchronized Set<GamePieceOnFieldSimulation> gamePiecesOnField() {
    return Set.copyOf(gamePiecesOnField);
  }

  /**
   * Returns all projectile pieces currently in flight.
   */
  public synchronized Set<GamePieceProjectile> gamePieceLaunched() {
    return Set.copyOf(gamePieceProjectiles);
  }

  /**
   * Returns all pieces (grounded + projectiles) filtered by type.
   */
  public synchronized List<GamePiece> getGamePiecesByType(String type) {
    Objects.requireNonNull(type);
    List<GamePiece> pieces = new ArrayList<>();
    for (GamePieceOnFieldSimulation grounded : gamePiecesOnField) {
      if (grounded.type().equals(type)) {
        pieces.add(grounded);
      }
    }
    for (GamePieceProjectile projectile : gamePieceProjectiles) {
      if (projectile.type().equals(type)) {
        pieces.add(projectile);
      }
    }
    return List.copyOf(pieces);
  }

  /**
   * Returns all piece poses by type as array for dashboard APIs.
   */
  public synchronized Pose3[] getGamePiecesArrayByType(String type) {
    List<Pose3> poses = getGamePiecesPosesByType(type);
    return poses.toArray(Pose3[]::new);
  }

  /**
   * Clears all active game pieces from field and projectile sets.
   */
  public synchronized void clearGamePieces() {
    gamePiecesOnField.clear();
    gamePieceProjectiles.clear();
  }

  /**
   * Resets field objects to autonomous starting state.
   */
  public synchronized void resetFieldForAuto() {
    clearGamePieces();
    placeGamePiecesOnField();
  }

  /**
   * Returns all registered goals.
   */
  public synchronized List<Goal> goals() {
    return List.copyOf(goals);
  }

  /**
   * Enables/disables match breakdown publishing.
   */
  public synchronized void setPublishMatchBreakdown(boolean enabled) {
    publishMatchBreakdown = enabled;
  }

  /**
   * Returns whether match breakdown publishing is enabled.
   */
  public synchronized boolean publishMatchBreakdownEnabled() {
    return publishMatchBreakdown;
  }

  /**
   * Updates match breakdown metric for one alliance.
   */
  protected synchronized void replaceValueInMatchBreakdown(boolean blueAlliance, String key,
      double value) {
    if (blueAlliance) {
      blueMatchBreakdown.put(key, value);
    } else {
      redMatchBreakdown.put(key, value);
    }
  }

  /**
   * Returns a published breakdown snapshot for both alliances.
   */
  public synchronized Map<String, Map<String, Double>> publishBreakdown() {
    Map<String, Map<String, Double>> out = new LinkedHashMap<>();
    out.put("blue", new LinkedHashMap<>(blueMatchBreakdown));
    out.put("red", new LinkedHashMap<>(redMatchBreakdown));
    out.get("blue").put("MatchClock", matchClockSeconds);
    out.get("red").put("MatchClock", matchClockSeconds);
    return out;
  }

  /**
   * Sets match clock in seconds.
   */
  public synchronized void setMatchClockSeconds(double matchClockSeconds) {
    this.matchClockSeconds = Math.max(0.0, matchClockSeconds);
  }

  /**
   * Gets current match clock in seconds.
   */
  public synchronized double matchClockSeconds() {
    return matchClockSeconds;
  }

  /**
   * Directly manipulates a grounded piece pose.
   */
  public synchronized void setGamePiecePose(GamePieceOnFieldSimulation piece, Pose2 pose) {
    Objects.requireNonNull(piece);
    piece.setPoseOnField(Objects.requireNonNull(pose));
  }

  /**
   * Directly manipulates a grounded piece velocity.
   */
  public synchronized void setGamePieceVelocity(GamePieceOnFieldSimulation piece, Vec3 velocityMps) {
    Objects.requireNonNull(piece);
    piece.setVelocity(Objects.requireNonNull(velocityMps));
  }

  /**
   * Applies an impulse-like velocity delta to a grounded piece.
   */
  public synchronized void applyGamePieceVelocityDelta(GamePieceOnFieldSimulation piece,
      Vec3 deltaVelocityMps) {
    Objects.requireNonNull(piece);
    Objects.requireNonNull(deltaVelocityMps);
    piece.setVelocity(piece.velocityMps().add(deltaVelocityMps));
  }

  /**
   * Gets all piece poses filtered by type for visual publishing.
   */
  public synchronized List<Pose3> getGamePiecesPosesByType(String type) {
    List<Pose3> poses = new ArrayList<>();
    for (GamePieceOnFieldSimulation grounded : gamePiecesOnField) {
      if (grounded.type().equals(type)) {
        poses.add(grounded.pose());
      }
    }
    for (GamePieceProjectile projectile : gamePieceProjectiles) {
      if (projectile.type().equals(type)) {
        poses.add(projectile.pose());
      }
    }
    return List.copyOf(poses);
  }

  /**
   * Maple-style alias for pose query by type.
   */
  public synchronized List<Pose3> getGamePiecePosesByType(String type) {
    return getGamePiecesPosesByType(type);
  }

  /**
   * Returns active projectile pieces.
   */
  public synchronized Set<GamePieceProjectile> getGamePieceProjectiles() {
    return gamePieceLaunched();
  }

  /**
   * Provides access to the underlying world for advanced integrations.
   */
  public PhysicsWorld world() {
    return world;
  }

  /**
   * Returns the simulation options backing this arena.
   */
  public SimulationOptions simulationOptions() {
    return options;
  }

  /**
   * Convenience helper for creating rigid body wrappers in arena context.
   */
  protected RigidBody createRigidBody(double massKg, Pose2 pose) {
    PhysicsBody body = world.createBody(massKg);
    body.setPosition(new Vec3(pose.xMeters(), pose.yMeters(), 0.0));
    body.setGravityEnabled(false);
    registerRoundBody(body, 0.45);
    return new RigidBody(body);
  }

  void registerRoundBody(PhysicsBody body, double radiusMeters) {
    if (!(radiusMeters > 0.0)) {
      throw new IllegalArgumentException("radiusMeters must be > 0");
    }
    roundBodies.add(new RoundBody(Objects.requireNonNull(body), radiusMeters));
  }

  private void applyBoundaryConstraints() {
    double width = options.boundaries().widthMeters();
    double height = options.boundaries().heightMeters();
    double restitution = options.collision().defaultRestitution();
    double tangentialDamping = options.friction().boundaryTangentialDamping();
    double velocityDeadband = options.tolerances().velocityDeadbandMps();

    for (RoundBody round : roundBodies) {
      Vec3 p = round.body().position();
      Vec3 v = round.body().linearVelocity();
      double r = round.radiusMeters();

      double x = p.x();
      double y = p.y();
      double vx = v.x();
      double vy = v.y();

      if (x - r < 0.0) {
        x = r;
        vx = Math.abs(vx) * restitution;
        vy *= (1.0 - tangentialDamping);
      } else if (x + r > width) {
        x = width - r;
        vx = -Math.abs(vx) * restitution;
        vy *= (1.0 - tangentialDamping);
      }

      if (y - r < 0.0) {
        y = r;
        vy = Math.abs(vy) * restitution;
        vx *= (1.0 - tangentialDamping);
      } else if (y + r > height) {
        y = height - r;
        vy = -Math.abs(vy) * restitution;
        vx *= (1.0 - tangentialDamping);
      }

      if (Math.abs(vx) <= velocityDeadband) {
        vx = 0.0;
      }
      if (Math.abs(vy) <= velocityDeadband) {
        vy = 0.0;
      }

      round.body().setPosition(new Vec3(x, y, 0.0));
      round.body().setLinearVelocity(new Vec3(vx, vy, 0.0));
    }
  }

  private Pose2 defaultIntakePose() {
    if (driveTrainSimulations.isEmpty()) {
      return new Pose2(0.0, 0.0, 0.0);
    }
    return driveTrainSimulations.get(0).pose();
  }

  private double totalScoreBlue() {
    int sum = 0;
    for (Goal goal : goals) {
      if (goal.ownerAlliance() == Alliance.BLUE) {
        sum += goal.scoredCount();
      }
    }
    return sum;
  }

  private double totalScoreRed() {
    int sum = 0;
    for (Goal goal : goals) {
      if (goal.ownerAlliance() == Alliance.RED) {
        sum += goal.scoredCount();
      }
    }
    return sum;
  }
}