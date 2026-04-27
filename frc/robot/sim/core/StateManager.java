package frc.robot.sim.core;

import frc.robot.sim.api.GamePieceState;
import frc.robot.sim.api.GamePieceType;
import edu.wpi.first.math.geometry.Rotation3d;

import java.util.*;

public class StateManager {

    private final Map<String, List<GamePieceState>> robotInventory = new HashMap<>();
    private final Map<String, Integer> robotCapacity = new HashMap<>();
    private final Map<GamePieceType, PieceConfig> pieceConfigs = new HashMap<>();

    private final List<GamePieceState> fieldPieces = new ArrayList<>();
    private final Map<GamePieceType, Integer> fieldCounts = new HashMap<>();

    public static class PieceConfig {
        public int maxOnField = Integer.MAX_VALUE;
        public int maxSpawnTotal = Integer.MAX_VALUE;
        public int spawnedSoFar = 0;
    }

    public void setRobotCapacity(String robotId, int capacity) {
        robotCapacity.put(robotId, capacity);
    }

    public int getRobotCapacity(String robotId) {
        return robotCapacity.getOrDefault(robotId, Integer.MAX_VALUE);
    }

    public void setPieceConfig(GamePieceType type, PieceConfig config) {
        pieceConfigs.put(type, config);
    }

    public int getHeldCount(String robotId) {
        return robotInventory.getOrDefault(robotId, List.of()).size();
    }

    public int getFieldCount(GamePieceType type) {
        return fieldCounts.getOrDefault(type, 0);
    }

    private void incrementField(GamePieceType type) {
        fieldCounts.put(type, getFieldCount(type) + 1);
    }

    private void decrementField(GamePieceType type) {
        fieldCounts.put(type, Math.max(0, getFieldCount(type) - 1));
    }

    public boolean intake(String robotId, GamePieceState piece) {
        robotInventory.putIfAbsent(robotId, new ArrayList<>());
        List<GamePieceState> held = robotInventory.get(robotId);

        if (held.size() >= getRobotCapacity(robotId)) return false;

        fieldPieces.remove(piece);
        decrementField(piece.getType());

        held.add(piece);
        return true;
    }

    public boolean outtake(String robotId, GamePieceType type, double velocity, Rotation3d rotation) {
        List<GamePieceState> held = robotInventory.get(robotId);
        if (held == null || held.isEmpty()) return false;

        Iterator<GamePieceState> it = held.iterator();

        while (it.hasNext()) {
            GamePieceState p = it.next();

            if (p.getType() == type) {
                it.remove();

                GamePieceState spawned = spawnFieldPiece(type, velocity, rotation);
                if (spawned == null) return false;

                fieldPieces.add(spawned);
                return true;
            }
        }

        return false;
    }

    private GamePieceState spawnFieldPiece(GamePieceType type, double velocity, Rotation3d rotation) {
        PieceConfig cfg = pieceConfigs.get(type);

        if (cfg != null) {
            if (cfg.spawnedSoFar >= cfg.maxSpawnTotal) return null;
            if (getFieldCount(type) >= cfg.maxOnField) return null;

            cfg.spawnedSoFar++;
        }

        GamePieceState piece = new GamePieceState(type);

        piece.setVelocity(velocity);
        piece.setRotation(rotation);

        incrementField(type);

        return piece;
    }
}
