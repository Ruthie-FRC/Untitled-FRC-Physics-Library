package rensim.simulation.gamepiece;

import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;

/**
 * Registry for game-piece definitions used by season arenas and manipulators.
 */
public final class GamePieceCatalog {
  private final Map<String, GamePieceData> byType = new LinkedHashMap<>();

  public GamePieceCatalog register(GamePieceData data) {
    Objects.requireNonNull(data);
    byType.put(data.type(), data);
    return this;
  }

  public Optional<GamePieceData> find(String type) {
    Objects.requireNonNull(type);
    return Optional.ofNullable(byType.get(type));
  }

  public GamePieceData require(String type) {
    return find(type).orElseThrow(() -> new IllegalArgumentException("Unknown game piece type: " + type));
  }

  public Collection<GamePieceData> all() {
    return byType.values();
  }
}
