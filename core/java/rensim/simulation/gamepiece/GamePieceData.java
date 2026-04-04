package rensim.simulation.gamepiece;

import java.util.Objects;
import rensim.simulation.GamePieceOnFieldSimulation;

/**
 * Canonical game piece metadata for simulation and launch/manipulation APIs.
 */
public record GamePieceData(
    String type,
    double massKg,
    double radiusMeters,
    double linearDamping,
    double angularDamping,
    double restitution,
    boolean launchable,
    boolean intakable,
    boolean manipulatable) {

  public GamePieceData {
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
    if (restitution < 0.0 || restitution > 1.0) {
      throw new IllegalArgumentException("restitution must be in [0, 1]");
    }
  }

  public static GamePieceData fromInfo(GamePieceOnFieldSimulation.GamePieceInfo info) {
    Objects.requireNonNull(info);
    return new GamePieceData(
        info.type(),
        info.massKg(),
        info.radiusMeters(),
        info.linearDamping(),
        info.angularDamping(),
        info.coefficientOfRestitution(),
        true,
        true,
        true);
  }

  public GamePieceOnFieldSimulation.GamePieceInfo toInfo() {
    return new GamePieceOnFieldSimulation.GamePieceInfo(
        type,
        massKg,
        radiusMeters,
        linearDamping,
        angularDamping,
        restitution);
  }
}
