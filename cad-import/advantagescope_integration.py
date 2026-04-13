"""
Optional visualization adapters for JSim arena snapshots.

JSim's arena state should be the source of truth. This module provides
adapter utilities to export a state snapshot into formats that external
visualization tools can consume, including NetworkTables-style keys.

Supports:
- Robot poses
- Game piece positions
- Field elements
- Generic JSON snapshot export
"""

import json
from typing import Dict, List, Any, Optional, Union
from dataclasses import asdict
import logging

from .arena_state import ArenaState, JSimStateTracker

logger = logging.getLogger(__name__)


class NetworkTablesKeyFormat:
    """Standard NetworkTables key formats for adapter-based integration."""
    
    # Root path for JSim simulation
    ROOT = "/jsim"
    
    # Arena state
    ARENA_TIME = f"{ROOT}/arena/time"
    ARENA_PAUSED = f"{ROOT}/arena/paused"
    ARENA_FIELD_LENGTH = f"{ROOT}/arena/field_length"
    ARENA_FIELD_WIDTH = f"{ROOT}/arena/field_width"
    
    # Robots (team_number substitution)
    ROBOT_POSE = f"{ROOT}/robots/{{team}}/pose"
    ROBOT_VELOCITY = f"{ROOT}/robots/{{team}}/velocity"
    ROBOT_ACTIVE = f"{ROOT}/robots/{{team}}/active"
    
    # Game pieces (piece_id substitution)
    GAMEPIECE_POSE = f"{ROOT}/game_pieces/{{id}}/pose"
    GAMEPIECE_VELOCITY = f"{ROOT}/game_pieces/{{id}}/velocity"
    GAMEPIECE_TYPE = f"{ROOT}/game_pieces/{{id}}/type"
    GAMEPIECE_ACTIVE = f"{ROOT}/game_pieces/{{id}}/active"
    
    # Field elements (element_name substitution)
    ELEMENT_POSE = f"{ROOT}/field_elements/{{name}}/pose"
    ELEMENT_TYPE = f"{ROOT}/field_elements/{{name}}/type"


class AdvantageeScopeExporter:
    """Exports arena state to optional visualization formats."""

    @staticmethod
    def arena_to_snapshot_dict(arena_state: Union[Dict[str, Any], ArenaState]) -> Dict[str, Any]:
        """Return a minimal tool-agnostic snapshot dictionary.

        Args:
            arena_state: Arena state snapshot

        Returns:
            Dictionary preserving JSim-oriented state structure
        """
        if hasattr(arena_state, "get_state_snapshot"):
            arena_state = arena_state.get_state_snapshot()

        return {
            "time": arena_state.get("time", 0.0),
            "paused": arena_state.get("paused", False),
            "field": arena_state.get("field", {}),
            "robots": arena_state.get("robots", []),
            "game_pieces": arena_state.get("game_pieces", []),
            "field_elements": arena_state.get("field_elements", []),
        }
    
    @staticmethod
    def arena_to_networktables_dict(arena_state: Dict[str, Any]) -> Dict[str, Any]:
        """Backward-compatible alias for the plain arena snapshot.
        
        Args:
            arena_state: Arena state snapshot
        
        Returns:
            Tool-agnostic arena snapshot dictionary
        """
        return AdvantageeScopeExporter.arena_to_snapshot_dict(arena_state)
    
    @staticmethod
    def export_to_nt_json(arena_state: Dict[str, Any], output_path: str) -> bool:
        """Legacy alias for exporting the plain arena snapshot JSON.
        
        Args:
            arena_state: Arena state snapshot
            output_path: Path to output JSON
        
        Returns:
            True if successful
        """
        try:
            snapshot = AdvantageeScopeExporter.arena_to_snapshot_dict(arena_state)
            
            with open(output_path, 'w') as f:
                json.dump(snapshot, f, indent=2)
            
            logger.info(f"Exported JSim snapshot to {output_path}")
            return True
        except Exception as e:
            logger.error(f"Failed to export snapshot JSON: {e}")
            return False

    @staticmethod
    def export_snapshot_json(arena_state: Dict[str, Any], output_path: str) -> bool:
        """Export a tool-agnostic snapshot JSON.

        This is the recommended default export for integrations.

        Args:
            arena_state: Arena state snapshot
            output_path: Path to output JSON

        Returns:
            True if successful
        """
        try:
            snapshot = AdvantageeScopeExporter.arena_to_snapshot_dict(arena_state)

            with open(output_path, 'w') as f:
                json.dump(snapshot, f, indent=2)

            logger.info(f"Exported JSim snapshot to {output_path}")
            return True
        except Exception as e:
            logger.error(f"Failed to export snapshot JSON: {e}")
            return False


class AdvantageKitVisualizer:
    """Generates optional code for AdvantageKit integration with JSim.
    
    Produces Java code that can be used in robot code to visualize
    JSim simulation data in AdvantageScope.
    """
    
    @staticmethod
    def generate_note_visualizer_java(
        output_path: str,
        game_year: int = 2024
    ) -> bool:
        """Generate Java code for note/game piece visualization.
        
        Based on:
        https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/example_projects/kitbot_2024/src/main/java/frc/robot/util/NoteVisualizer.java
        
        Args:
            output_path: Path to output Java file
            game_year: FRC game year
        
        Returns:
            True if successful
        """
        code = f'''package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import org.littletonrobotics.junction.Logger;
import java.util.ArrayList;
import java.util.List;

/**
    * OPTIONAL: Auto-generated visualizer for JSim game pieces in {game_year}.
 * Publishes game piece poses to AdvantageScope via NetworkTables.
 */
public class GamePieceVisualizer {{
    
    private static final String NT_ROOT = "/jsim/game_pieces";
    
    /**
     * Update game piece visualization in AdvantageScope.
     * 
     * Call this at ~20ms intervals (in robotPeriodic or main loop).
     * 
     * @param gamePieces List of game piece poses
     */
    public static void updateGamePieces(List<Pose3d> gamePieces) {{
        // Create array of poses for AdvantageScope
        Pose3d[] poses = gamePieces.toArray(new Pose3d[0]);
        
        // Optional AdvantageKit logger integration.
        Logger.recordOutput(NT_ROOT + "/poses", poses);
    }}
    
    /**
     * Optional robot pose output for visualization.
     */
    public static void updateRobotPose(Pose3d robotPose) {{
        Logger.recordOutput("/jsim/robot/pose", robotPose);
    }}
    
    /**
     * Example: Create a pose from JSim data
     */
    public static Pose3d fromJSimData(
        double x, double y, double z,
        double roll, double pitch, double yaw
    ) {{
        return new Pose3d(
            new Translation3d(x, y, z),
            new Rotation3d(roll, pitch, yaw)
        );
    }}
}}
'''
        
        try:
            with open(output_path, 'w') as f:
                f.write(code)
            logger.info(f"Generated AdvantageKit visualizer: {output_path}")
            return True
        except Exception as e:
            logger.error(f"Failed to generate visualizer: {e}")
            return False
    
    @staticmethod
    def generate_pose3d_updater_java(output_path: str) -> bool:
        """Generate Java code for updating Pose3d in robot code.
        
        Args:
            output_path: Path to output Java file
        
        Returns:
            True if successful
        """
        code = '''package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Optional utility for updating Pose3d via NetworkTables from JSim.
 * 
 * Subscribes to JSim-published poses and updates robot state.
 */
public class JSImPose3dUpdater implements Runnable {{
    
    private static final String NT_PREFIX = "/jsim";
    private NetworkTable table;
    private DoubleArrayTopic poseTopic;
    private volatile Pose3d currentPose;
    
    public JSImPose3dUpdater() {{
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        table = inst.getTable(NT_PREFIX);
        
        // Subscribe to robot pose updates
        poseTopic = table.getDoubleArrayTopic("robot/pose");
        currentPose = new Pose3d();
    }}
    
    /**
     * Update pose from NetworkTables.
     * Expected format: [x, y, z, roll, pitch, yaw]
     */
    public void updatePose() {{
        double[] poseArray = poseTopic.getEntry(new double[6]).get();
        
        if (poseArray.length == 6) {{
            currentPose = new Pose3d(
                new Translation3d(poseArray[0], poseArray[1], poseArray[2]),
                new Rotation3d(poseArray[3], poseArray[4], poseArray[5])
            );
        }}
    }}
    
    /**
     * Get current pose (thread-safe).
     */
    public synchronized Pose3d getPose() {{
        return currentPose;
    }}
    
    @Override
    public void run() {{
        // High-frequency update loop (50Hz)
        while (!Thread.currentThread().isInterrupted()) {{
            updatePose();
            
            try {{
                Thread.sleep(20);  // 20ms cycle
            }} catch (InterruptedException e) {{
                Thread.currentThread().interrupt();
                break;
            }}
        }}
    }}
}}
'''
        
        try:
            with open(output_path, 'w') as f:
                f.write(code)
            logger.info(f"Generated Pose3d updater: {output_path}")
            return True
        except Exception as e:
            logger.error(f"Failed to generate updater: {e}")
            return False
