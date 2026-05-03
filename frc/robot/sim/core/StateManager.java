import api.GamePieceState;
import api.GamePieceType;
import api.Rotation3d;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;

/**
 * @file StateManager.java
 * @brief Central manager for game piece state across robots and the field.
 *
 * Handles:
 * - Robot inventories (what each robot is holding)
 * - Capacity limits per robot
 * - Field piece tracking and counts
 * - Spawn limits and lifecycle of game pieces
 *
 * This class acts as the authoritative source for all game piece transitions
 * between robots and the field.
 */
public class StateManager {

    /** Maps robot IDs to their held game pieces */
    private final Map<String, List<GamePieceState>> robotInventory = new HashMap<>();

    /** Stores per-robot capacity limits */
    private final Map<String, Integer> robotCapacity = new HashMap<>();

    /** Configuration per game piece type */
    private final Map<GamePieceType, PieceConfig> pieceConfigs = new HashMap<>();

    /** All game pieces currently on the field */
    private final List<GamePieceState> fieldPieces = new ArrayList<>();

    /** Count of each game piece type currently on the field */
    private final Map<GamePieceType, Integer> fieldCounts = new HashMap<>();

    /**
     * @brief Configuration for a specific game piece type.
     */
    public static class PieceConfig {
        /** Maximum number allowed simultaneously on the field */
        public int maxOnField = Integer.MAX_VALUE;

        /** Maximum total number that can ever be spawned */
        public int maxSpawnTotal = Integer.MAX_VALUE;

        /** Total number spawned so far */
        public int spawnedSoFar = 0;

        /** Current number alive on the field (not strictly enforced here) */
        public int aliveCount = 0;
    }

    /**
     * @brief Sets the maximum capacity for a robot.
     * @param robotId Unique robot identifier
     * @param capacity Maximum number of pieces the robot can hold
     */
    public void setRobotCapacity(String robotId, int capacity) {
        robotCapacity.put(robotId, capacity);
    }

    /**
     * @brief Gets the maximum capacity for a robot.
     * @param robotId Unique robot identifier
     * @return Capacity, or unlimited if not set
     */
    public int getRobotCapacity(String robotId) {
        return robotCapacity.getOrDefault(robotId, Integer.MAX_VALUE);
    }

    /**
     * @brief Sets configuration for a game piece type.
     * @param type Game piece type
     * @param config Configuration object
     */
    public void setPieceConfig(GamePieceType type, PieceConfig config) {
        pieceConfigs.put(type, config);
    }

    /**
     * @brief Gets how many pieces a robot is currently holding.
     * @param robotId Unique robot identifier
     * @return Number of held pieces
     */
    public int getHeldCount(String robotId) {
        return robotInventory.getOrDefault(robotId, List.of()).size();
    }

    /**
     * @brief Gets number of pieces of a given type on the field.
     * @param type Game piece type
     * @return Count on field
     */
    public int getFieldCount(GamePieceType type) {
        return fieldCounts.getOrDefault(type, 0);
    }

    /** Increment field count for a piece type */
    private void incrementField(GamePieceType type) {
        fieldCounts.put(type, getFieldCount(type) + 1);
    }

    /** Decrement field count for a piece type */
    private void decrementField(GamePieceType type) {
        fieldCounts.put(type, Math.max(0, getFieldCount(type) - 1));
    }

    /**
     * @brief Intake a game piece into a robot.
     *
     * Removes the piece from the field and adds it to the robot's inventory
     * if capacity allows.
     *
     * @param robotId Unique robot identifier
     * @param piece Game piece to intake
     * @return True if successful, false if capacity exceeded
     */
    public boolean intake(String robotId, GamePieceState piece) {
        robotInventory.putIfAbsent(robotId, new ArrayList<>());
        List<GamePieceState> held = robotInventory.get(robotId);

        if (held.size() >= getRobotCapacity(robotId)) {
            return false;
        }

        fieldPieces.remove(piece);
        decrementField(piece.getType());
        held.add(piece);
        return true;
    }

    /**
     * @brief Outtake a game piece from a robot onto the field.
     *
     * Removes a matching piece from the robot's inventory and spawns a new
     * field piece with the given velocity and rotation.
     *
     * @param robotId Unique robot identifier
     * @param type Type of piece to eject
     * @param velocity Initial velocity
     * @param rotation Initial rotation
     * @return True if successful, false otherwise
     */
    public boolean outtake(String robotId, GamePieceType type, double velocity, Rotation3d rotation) {
        List<GamePieceState> held = robotInventory.get(robotId);

        if (held == null || held.isEmpty()) {
            return false;
        }

        Iterator<GamePieceState> it = held.iterator();
        while (it.hasNext()) {
            GamePieceState p = it.next();
            if (p.getType() == type) {
                it.remove();

                GamePieceState spawned = spawnFieldPiece(type, velocity, rotation);
                if (spawned == null) {
                    return false;
                }

                fieldPieces.add(spawned);
                return true;
            }
        }
        return false;
    }

    /**
     * @brief Spawns a new game piece on the field if allowed.
     *
     * Enforces per-type limits on total spawns and concurrent field presence.
     *
     * @param type Game piece type
     * @param velocity Initial velocity
     * @param rotation Initial rotation
     * @return New GamePieceState, or null if limits prevent spawning
     */
    private GamePieceState spawnFieldPiece(GamePieceType type, double velocity, Rotation3d rotation) {
        PieceConfig cfg = pieceConfigs.get(type);

        if (cfg != null) {
            if (cfg.spawnedSoFar >= cfg.maxSpawnTotal) {
                return null;
            }
            if (getFieldCount(type) >= cfg.maxOnField) {
                return null;
            }
            cfg.spawnedSoFar++;
        }

        GamePieceState piece = new GamePieceState(type);
        piece.setVelocity(velocity);
        piece.setRotation(rotation);
        incrementField(type);
        return piece;
    }

    /** Load native simulation library */
    static {
        System.loadLibrary("jsimcore");
    }
}
