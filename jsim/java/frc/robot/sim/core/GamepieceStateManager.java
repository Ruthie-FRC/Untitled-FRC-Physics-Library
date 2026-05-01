package frc.robot.sim.core;

import api.GamePieceState;
import api.GamePieceType;
import api.Rotation3d;
import api.RobotID;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;

public class GamepieceStateManager {

    private final Map<RobotID, List<GamePieceState>> robotInventory = new HashMap<>();
    private final Map<RobotID, Integer> robotCapacity = new HashMap<>();
    private final Map<GamePieceType, PieceConfig> pieceConfigs = new HashMap<>();
    private final List<GamePieceState> fieldPieces = new ArrayList<>();
    private final Map<GamePieceType, Integer> fieldCounts = new HashMap<>();

    public static class PieceConfig {
        public int maxOnField = Integer.MAX_VALUE;
        public int maxSpawnTotal = Integer.MAX_VALUE;
        public int spawnedSoFar = 0;
    }

    // =========================
    // Robot config
    // =========================

    public void setRobotCapacity(RobotID robotId, int capacity) {
        robotCapacity.put(robotId, capacity);
    }

    public int getRobotCapacity(RobotID robotId) {
        return robotCapacity.getOrDefault(robotId, Integer.MAX_VALUE);
    }

    public int getHeldCount(RobotID robotId) {
        return robotInventory.getOrDefault(robotId, List.of()).size();
    }

    // =========================
    // Config
    // =========================

    public void setPieceConfig(GamePieceType type, PieceConfig config) {
        pieceConfigs.put(type, config);
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

    // =========================
    // Intake
    // =========================

    public boolean intake(RobotID robotId, GamePieceState piece) {
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

    // =========================
    // Outtake
    // =========================

    public boolean outtake(RobotID robotId, GamePieceType type, double velocity, Rotation3d rotation) {
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

    // =========================
    // Spawning
    // =========================

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
}
