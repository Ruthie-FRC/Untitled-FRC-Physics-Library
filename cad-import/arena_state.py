"""
Arena and field state management for JSim.

Tracks robots, game pieces, and field elements with thread-safe updates
for integration with AdvantageScope visualization.

From Issue #45: Each season requires field recreation with material interactions,
component definitions, and efficient state tracking.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Any
from threading import RLock
from enum import Enum
import json
import logging

logger = logging.getLogger(__name__)


class GamePieceType(Enum):
    """Types of game pieces that can exist on field."""
    NOTE = "note"
    BALL = "ball"
    CUBE = "cube"
    CONE = "cone"
    RING = "ring"
    GAMEPIECE = "gamepiece"


@dataclass
class Pose3d:
    """Represents a 3D pose (position + rotation).
    
    Compatible with WPILib Pose3d for NetworkTables export.
    """
    x: float  # meters
    y: float  # meters
    z: float  # meters
    roll: float  # radians
    pitch: float  # radians
    yaw: float  # radians
    
    def to_dict(self) -> Dict[str, float]:
        """Convert to dictionary for JSON/NetworkTables."""
        return {
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "roll": self.roll,
            "pitch": self.pitch,
            "yaw": self.yaw,
        }
    
    @staticmethod
    def from_dict(data: Dict[str, float]) -> 'Pose3d':
        """Create from dictionary."""
        return Pose3d(
            x=data.get("x", 0),
            y=data.get("y", 0),
            z=data.get("z", 0),
            roll=data.get("roll", 0),
            pitch=data.get("pitch", 0),
            yaw=data.get("yaw", 0),
        )


@dataclass
class GamePiece:
    """Represents a game piece on the field."""
    id: str  # Unique identifier
    type: GamePieceType
    pose: Pose3d
    velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    mass: float = 0.1  # kg
    material: str = "rubber"
    active: bool = True  # Still in play
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for export."""
        return {
            "id": self.id,
            "type": self.type.value,
            "pose": self.pose.to_dict(),
            "velocity": {"x": self.velocity[0], "y": self.velocity[1], "z": self.velocity[2]},
            "mass": self.mass,
            "material": self.material,
            "active": self.active,
        }


@dataclass
class Robot:
    """Represents a robot on the field."""
    team_number: int
    alliance: str  # "red" or "blue"
    pose: Pose3d
    velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    angular_velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    active: bool = True
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for export."""
        return {
            "team_number": self.team_number,
            "alliance": self.alliance,
            "pose": self.pose.to_dict(),
            "velocity": {"x": self.velocity[0], "y": self.velocity[1], "z": self.velocity[2]},
            "angular_velocity": {"x": self.angular_velocity[0], "y": self.angular_velocity[1], "z": self.angular_velocity[2]},
            "active": self.active,
        }


@dataclass
class FieldElement:
    """Represents a static/dynamic field element (Goal, platform, etc.)."""
    name: str
    element_type: str  # "goal", "platform", "frame", etc.
    pose: Pose3d
    dimensions: Dict[str, float] = field(default_factory=dict)
    material: str = "aluminum"
    mass: float = 5.0
    collision_enabled: bool = True
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for export."""
        return {
            "name": self.name,
            "type": self.element_type,
            "pose": self.pose.to_dict(),
            "dimensions": self.dimensions,
            "material": self.material,
            "mass": self.mass,
            "collision_enabled": self.collision_enabled,
        }


class ArenaState:
    """Thread-safe arena state management.
    
    Tracks all game pieces and robots with efficient preallocated storage.
    Designed for high-frequency updates (20ms refresh rate).
    """
    
    def __init__(self, field_length: float = 16.54, field_width: float = 8.21):
        """Initialize arena.
        
        Args:
            field_length: Field length in meters (default: 2024 FRC field)
            field_width: Field width in meters
        """
        self.field_length = field_length
        self.field_width = field_width
        
        # Thread safety
        self._lock = RLock()
        
        # Preallocated storage (avoids mallocs)
        self._max_game_pieces = 256  # Worst case: many balls
        self._max_robots = 12  # 6 per alliance
        self._max_field_elements = 128
        
        # Current state
        self.game_pieces: Dict[str, GamePiece] = {}
        self.robots: Dict[int, Robot] = {}
        self.field_elements: Dict[str, FieldElement] = {}
        
        # Active indices (avoid linked lists)
        self._active_piece_ids: List[str] = []
        self._active_robot_ids: List[int] = []
        self._active_element_ids: List[str] = []
        
        self.simulation_time: float = 0.0  # seconds
        self.paused: bool = False
        
        logger.info(f"Arena initialized: {field_length}m x {field_width}m")
    
    def add_game_piece(self, piece: GamePiece) -> bool:
        """Add a game piece to the arena.
        
        Args:
            piece: GamePiece to add
        
        Returns:
            True if added, False if arena full
        """
        with self._lock:
            if len(self.game_pieces) >= self._max_game_pieces:
                logger.warning(f"Arena full: cannot add game piece {piece.id}")
                return False
            
            self.game_pieces[piece.id] = piece
            if piece.active:
                self._active_piece_ids.append(piece.id)
            
            logger.debug(f"Added game piece: {piece.id} ({piece.type.value})")
            return True
    
    def update_game_piece(self, piece_id: str, pose: Pose3d, velocity: Tuple[float, float, float]):
        """Update game piece pose and velocity.
        
        Args:
            piece_id: ID of piece to update
            pose: New pose
            velocity: New velocity
        """
        with self._lock:
            if piece_id not in self.game_pieces:
                logger.warning(f"Game piece not found: {piece_id}")
                return
            
            piece = self.game_pieces[piece_id]
            piece.pose = pose
            piece.velocity = velocity
    
    def remove_game_piece(self, piece_id: str) -> bool:
        """Remove a game piece from the arena.
        
        Args:
            piece_id: ID of piece to remove
        
        Returns:
            True if removed
        """
        with self._lock:
            if piece_id not in self.game_pieces:
                return False
            
            del self.game_pieces[piece_id]
            if piece_id in self._active_piece_ids:
                self._active_piece_ids.remove(piece_id)
            
            logger.debug(f"Removed game piece: {piece_id}")
            return True
    
    def add_robot(self, robot: Robot) -> bool:
        """Add a robot to the arena.
        
        Args:
            robot: Robot to add
        
        Returns:
            True if added
        """
        with self._lock:
            if robot.team_number in self.robots:
                logger.warning(f"Robot team {robot.team_number} already on field")
                return False
            
            self.robots[robot.team_number] = robot
            if robot.active:
                self._active_robot_ids.append(robot.team_number)
            
            logger.info(f"Added robot: Team {robot.team_number} ({robot.alliance})")
            return True
    
    def update_robot_pose(self, team_number: int, pose: Pose3d, velocity: Tuple[float, float, float]):
        """Update robot pose and velocity.
        
        Args:
            team_number: Team number
            pose: New pose
            velocity: New velocity
        """
        with self._lock:
            if team_number not in self.robots:
                logger.warning(f"Robot not found: Team {team_number}")
                return
            
            robot = self.robots[team_number]
            robot.pose = pose
            robot.velocity = velocity
    
    def add_field_element(self, element: FieldElement) -> bool:
        """Add a field element.
        
        Args:
            element: FieldElement to add
        
        Returns:
            True if added
        """
        with self._lock:
            if len(self.field_elements) >= self._max_field_elements:
                logger.warning(f"Too many field elements")
                return False
            
            self.field_elements[element.name] = element
            self._active_element_ids.append(element.name)
            
            logger.debug(f"Added field element: {element.name} ({element.element_type})")
            return True
    
    def get_active_game_pieces(self) -> List[GamePiece]:
        """Get list of active game pieces (efficient iteration).
        
        Returns:
            List of active game pieces
        """
        with self._lock:
            return [self.game_pieces[pid] for pid in self._active_piece_ids if pid in self.game_pieces]
    
    def get_active_robots(self) -> List[Robot]:
        """Get list of active robots.
        
        Returns:
            List of active robots
        """
        with self._lock:
            return [self.robots[tid] for tid in self._active_robot_ids if tid in self.robots]
    
    def get_state_snapshot(self) -> Dict[str, Any]:
        """Get snapshot of arena state for export/visualization.
        
        Returns:
            Dictionary with complete arena state
        """
        with self._lock:
            return {
                "time": self.simulation_time,
                "paused": self.paused,
                "field": {
                    "length": self.field_length,
                    "width": self.field_width,
                },
                "robots": [r.to_dict() for r in self.get_active_robots()],
                "game_pieces": [p.to_dict() for p in self.get_active_game_pieces()],
                "field_elements": [e.to_dict() for e in self.field_elements.values()],
            }
    
    def step(self, dt: float):
        """Advance simulation by time step.
        
        Args:
            dt: Time step in seconds
        """
        with self._lock:
            if not self.paused:
                self.simulation_time += dt
    
    def pause(self):
        """Pause simulation."""
        with self._lock:
            self.paused = True
    
    def resume(self):
        """Resume simulation."""
        with self._lock:
            self.paused = False
    
    def reset(self):
        """Reset arena to initial state."""
        with self._lock:
            self.game_pieces.clear()
            self.robots.clear()
            self.field_elements.clear()
            self._active_piece_ids.clear()
            self._active_robot_ids.clear()
            self._active_element_ids.clear()
            self.simulation_time = 0.0
            self.paused = False
            logger.info("Arena reset")
    
    def get_summary(self) -> Dict[str, Any]:
        """Get arena summary statistics."""
        with self._lock:
            return {
                "field_dimensions": f"{self.field_length}m x {self.field_width}m",
                "active_robots": len(self._active_robot_ids),
                "active_game_pieces": len(self._active_piece_ids),
                "field_elements": len(self._active_element_ids),
                "simulation_time": self.simulation_time,
                "paused": self.paused,
            }


class JSimStateTracker:
    """Owns arena snapshot export for JSim integrations.

    This keeps the state-tracking contract in the core arena-state module so
    integrations can consume a stable snapshot without adding user-code glue.
    """

    REQUIREMENTS = (
        "ArenaState snapshot access",
        "Periodic export or polling from JSim runtime",
    )

    INTEGRATIONS = (
        "snapshot_dict",
        "snapshot_json",
    )

    def __init__(self, arena_state: ArenaState):
        self.arena_state = arena_state

    def snapshot_dict(self) -> Dict[str, Any]:
        """Return the current arena snapshot."""
        return self.arena_state.get_state_snapshot()

    def export_snapshot_json(self, output_path: str) -> bool:
        """Export the current arena snapshot to JSON."""
        try:
            with open(output_path, 'w') as f:
                json.dump(self.snapshot_dict(), f, indent=2)
            return True
        except Exception as e:
            logger.error(f"Failed to export arena snapshot JSON: {e}")
            return False
