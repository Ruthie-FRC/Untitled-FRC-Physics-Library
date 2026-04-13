"""
Comprehensive examples for field and arena setup for JSim.

From Issue #45: Each season requires recreating the arena and game elements.
These examples demonstrate best practices for:
1. Importing field CAD
2. Setting up arena state
3. Initializing game pieces
4. Configuring material interactions
5. Integrating with AdvantageScope
"""

import json
from pathlib import Path
from typing import List, Dict, Any

# Import from our modules
from arena_state import (
    ArenaState, GamePiece, GamePieceType, Robot, FieldElement, Pose3d, JSimStateTracker
)
from field_definitions import FieldDefinitionManager, Field2024Definition


def example_1_basic_field_setup():
    """Example 1: Basic field setup for 2024 CRESCENDO."""
    print("\n" + "="*60)
    print("Example 1: Basic Field Setup (2024 CRESCENDO)")
    print("="*60)
    
    # Load field definition
    field_def = FieldDefinitionManager.get_field_definition(2024)
    print(f"✓ Loaded field definition for {field_def['game']}")
    
    # Create arena
    arena = ArenaState(
        field_length=Field2024Definition.FIELD_LENGTH,
        field_width=Field2024Definition.FIELD_WIDTH
    )
    print(f"✓ Created arena: {arena.field_length}m x {arena.field_width}m")
    
    # Add field elements from definition
    for element_data in field_def.get("elements", []):
        element = FieldElement(
            name=element_data["name"],
            element_type=element_data["type"],
            pose=Pose3d.from_dict(element_data["pose"]),
            dimensions=element_data.get("dimensions", {}),
            material=element_data.get("material", "aluminum"),
            mass=element_data.get("mass", 0.0),
        )
        arena.add_field_element(element)
    
    print(f"✓ Added {len(arena.field_elements)} field elements")
    
    # Get summary
    summary = arena.get_summary()
    print(f"\nArena Summary:")
    for key, value in summary.items():
        print(f"  {key}: {value}")


def example_2_add_robots():
    """Example 2: Add robots to the arena."""
    print("\n" + "="*60)
    print("Example 2: Adding Robots to Arena")
    print("="*60)
    
    # Create arena
    arena = ArenaState(
        field_length=Field2024Definition.FIELD_LENGTH,
        field_width=Field2024Definition.FIELD_WIDTH
    )
    
    # Add Blue Alliance robots
    blue_teams = [1690, 2910, 1234]  # Example team numbers
    blue_x_positions = [2.0, 2.0, 2.0]  # Staging line
    
    for i, team in enumerate(blue_teams):
        robot = Robot(
            team_number=team,
            alliance="blue",
            pose=Pose3d(
                x=blue_x_positions[i],
                y=1.0 + i * 2.0,  # Spread along field
                z=0.0,
                roll=0.0, pitch=0.0, yaw=0.0
            )
        )
        arena.add_robot(robot)
        print(f"✓ Added Blue robot: Team {team}")
    
    # Add Red Alliance robots
    red_teams = [4414, 5499, 3005]
    red_x_positions = [14.54, 14.54, 14.54]  # Red staging line
    
    for i, team in enumerate(red_teams):
        robot = Robot(
            team_number=team,
            alliance="red",
            pose=Pose3d(
                x=red_x_positions[i],
                y=1.0 + i * 2.0,
                z=0.0,
                roll=0.0, pitch=0.0, yaw=3.14159  # Face opposite direction
            )
        )
        arena.add_robot(robot)
        print(f"✓ Added Red robot: Team {team}")
    
    print(f"\nArena contains {len(arena.robots)} robots")


def example_3_add_game_pieces():
    """Example 3: Add game pieces to arena."""
    print("\n" + "="*60)
    print("Example 3: Adding Game Pieces (Notes)")
    print("="*60)
    
    arena = ArenaState()
    
    # Define initial note positions on field (2024)
    note_positions = [
        (2.0, 4.0),   # Blue side staging
        (2.5, 4.0),
        (3.0, 4.0),
        (13.54, 4.0), # Red side staging
        (14.0, 4.0),
        (14.5, 4.0),
        (8.27, 0.5),  # Center notes
        (8.27, 4.1),
        (8.27, 7.7),
    ]
    
    for i, (x, y) in enumerate(note_positions):
        note = GamePiece(
            id=f"note_{i:02d}",
            type=GamePieceType.NOTE,
            pose=Pose3d(x=x, y=y, z=0.2, roll=0, pitch=0, yaw=0),
            mass=0.235,  # kg
            material="rubber_composite",
        )
        arena.add_game_piece(note)
    
    print(f"✓ Added {len(arena.game_pieces)} notes to arena")
    
    # Simulate note being picked up
    print("\n→ Simulating note pickup...")
    arena.remove_game_piece("note_00")
    print(f"✓ Removed note_00 from arena")
    print(f"  Remaining notes: {len(arena.game_pieces)}")


def example_4_update_robot_pose():
    """Example 4: Update robot pose during simulation."""
    print("\n" + "="*60)
    print("Example 4: Updating Robot Pose During Simulation")
    print("="*60)
    
    arena = ArenaState()
    
    # Add robot
    robot = Robot(
        team_number=1690,
        alliance="blue",
        pose=Pose3d(x=2.0, y=4.1, z=0.0, roll=0, pitch=0, yaw=0),
    )
    arena.add_robot(robot)
    
    print(f"Initial position: ({robot.pose.x}, {robot.pose.y})")
    
    # Simulate robot movement
    dt = 0.02  # 20ms
    velocity = (1.0, 0.5, 0.0)  # m/s x, y, z
    
    for step in range(5):
        new_x = robot.pose.x + velocity[0] * dt
        new_y = robot.pose.y + velocity[1] * dt
        
        new_pose = Pose3d(
            x=new_x, y=new_y, z=0.0,
            roll=0, pitch=0, yaw=0
        )
        
        arena.update_robot_pose(1690, new_pose, velocity)
        arena.step(dt)
        
        print(f"Step {step+1}: pos=({new_x:.2f}, {new_y:.2f}), time={arena.simulation_time:.3f}s")


def example_5_export_to_advantagescope():
    """Example 5: Export arena state to AdvantageScope format."""
    print("\n" + "="*60)
    print("Example 5: Exporting to AdvantageScope")
    print("="*60)
    
    # Setup arena
    arena = ArenaState()
    
    # Add robot
    robot = Robot(
        team_number=1690,
        alliance="blue",
        pose=Pose3d(x=5.0, y=4.0, z=0.0, roll=0, pitch=0, yaw=0.5),
        velocity=(1.0, 0.5, 0.0),
    )
    arena.add_robot(robot)
    
    # Add note
    note = GamePiece(
        id="note_01",
        type=GamePieceType.NOTE,
        pose=Pose3d(x=6.0, y=4.0, z=0.2, roll=0, pitch=0, yaw=0),
    )
    arena.add_game_piece(note)
    
    # Export through the JSim-owned state tracker
    tracker = JSimStateTracker(arena)
    snapshot_dict = tracker.snapshot_dict()
    
    print(f"✓ Generated {len(snapshot_dict)} snapshot entries")
    
    # Show sample entries
    print("\nSample Snapshot Entries:")
    for i, (key, value) in enumerate(list(snapshot_dict.items())[:5]):
        print(f"  {key}: {value}")


def example_6_season_field_setup():
    """Example 6: Complete season setup - create all field definitions."""
    print("\n" + "="*60)
    print("Example 6: Complete Season Field Setup")
    print("="*60)
    
    # Save field definitions for all available years
    output_dir = Path("./field_definitions")
    output_dir.mkdir(exist_ok=True)
    
    for year in FieldDefinitionManager.list_available_years():
        filename = output_dir / f"field_{year}.json"
        
        if FieldDefinitionManager.save_field_definition(year, str(filename)):
            print(f"✓ Saved {year} field definition")
        
        # Load and verify
        field_def = FieldDefinitionManager.load_field_definition(str(filename))
        print(f"  - Game: {field_def.get('game', 'Unknown')}")
        print(f"  - Elements: {len(field_def.get('elements', []))}")


def example_7_material_interactions():
    """Example 7: Configure material interactions."""
    print("\n" + "="*60)
    print("Example 7: Material Interactions Configuration")
    print("="*60)
    
    # Material interaction matrix
    # [friction, restitution] between materials
    interactions = {
        ("rubber", "aluminum"): (0.4, 0.3),
        ("rubber", "wood"): (0.5, 0.25),
        ("rubber", "plastic"): (0.3, 0.4),
        ("aluminum", "aluminum"): (0.3, 0.2),
    }
    
    print("Material Interaction Matrix:")
    print(f"{'Material 1':<15} {'Material 2':<15} {'Friction':<10} {'Restitution':<10}")
    print("-" * 50)
    
    for (mat1, mat2), (friction, restitution) in interactions.items():
        print(f"{mat1:<15} {mat2:<15} {friction:<10.2f} {restitution:<10.2f}")
    
    # Save to JSON
    config = {
        "material_interactions": {
            f"{mat1}_{mat2}": {
                "friction": friction,
                "restitution": restitution
            }
            for (mat1, mat2), (friction, restitution) in interactions.items()
        }
    }
    
    with open("material_interactions.json", "w") as f:
        json.dump(config, f, indent=2)
    
    print("\n✓ Saved material interactions to material_interactions.json")


def example_8_performance_optimization():
    """Example 8: Performance optimization for high-frequency updates."""
    print("\n" + "="*60)
    print("Example 8: Performance Optimization Tips")
    print("="*60)
    
    tips = """
Guidelines for high-frequency updates (20ms+):

1. PREALLOCATE ARRAYS
   ✓ Use list comprehensions for batch operations
   ✓ Avoid repeated list.append() in tight loops
   
2. MINIMIZE ALLOCATIONS
   ✓ Reuse Pose3d objects when possible
   ✓ Use in-place updates for velocities
   ✗ Avoid temporary objects in inner loops
   
3. AVOID LINKED LISTS
   ✓ Use arrays/lists with indices
   ✗ LinkedList has pointer overhead
   
4. THREAD SAFETY
   ✓ Use RLock for state updates
   ✓ Minimize lock contention
   ✓ Keep lock-protected sections short
   
5. CACHE FREQUENTLY ACCESSED VALUES
   ✓ Store field_length, field_width at init
   ✓ Avoid dictionary lookups in tight loops
   
6. BATCH VISUALIZATION UPDATES
   ✓ Collect pose updates, publish once per cycle
   ✗ Publish each robot pose separately
    """
    
    print(tips)
    
    # Code example: Efficient update loop
    code_example = '''
# EFFICIENT: Batch arena updates
arena = ArenaState()
dt = 0.02

# Preallocate lists
robot_updates = []
piece_updates = []

# Collect all updates
for robot in active_robots:
    new_pose = calculate_new_pose(robot)
    robot_updates.append((robot.team_number, new_pose))

for piece in active_pieces:
    new_pose = calculate_new_pose(piece)
    piece_updates.append((piece.id, new_pose))

# Apply all at once with single lock
with arena._lock:
    for team_num, new_pose in robot_updates:
        arena.update_robot_pose(team_num, new_pose, velocity)
    
    for piece_id, new_pose in piece_updates:
        arena.update_game_piece(piece_id, new_pose, velocity)
    
    arena.step(dt)

# Export once per cycle
tracker = JSimStateTracker(arena)
tracker.export_snapshot_json(output_path)
    '''
    
    print("Efficient Update Loop Example:")
    print(code_example)


if __name__ == "__main__":
    print("="*60)
    print("JSim Arena and Field Setup Examples")
    print("Issue #45: Seasonal Field Recreation")
    print("="*60)
    
    try:
        example_1_basic_field_setup()
        example_2_add_robots()
        example_3_add_game_pieces()
        example_4_update_robot_pose()
        example_5_export_to_advantagescope()
        example_6_season_field_setup()
        example_7_material_interactions()
        example_8_performance_optimization()
        
        print("\n" + "="*60)
        print("✓ All examples completed!")
        print("="*60)
        
    except Exception as e:
        print(f"\n✗ Example failed: {e}")
        import traceback
        traceback.print_exc()
