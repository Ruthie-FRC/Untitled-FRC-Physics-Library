# Seasonal Field Recreation Guide

Complete guide for recreating FRC field definitions and game elements for JSim each season.

Scope note: `cad-import` is responsible for generating and validating season JSON.
Runtime state tracking, NT publishing, and live visualization should be implemented
inside the vendordep/runtime layer.

## Quick Start

### 1. Load Existing Field Definition

```python
from field_definitions import FieldDefinitionManager

# Get 2024 CRESCENDO field
field_def = FieldDefinitionManager.get_field_definition(2024)
print(f"Game: {field_def['game']}")
print(f"Elements: {len(field_def['elements'])}")
```

### 2. Validate Field JSON

```python
dims = field_def["field_dimensions"]
assert "length" in dims and "width" in dims
assert "elements" in field_def
assert "game_pieces" in field_def
```

### 3. Export JSON for Runtime Consumption

```python
import json

with open("field_2026.json", "w") as f:
    json.dump(field_def, f, indent=2)
```

### 4. Load in vdep/runtime

Use the exported JSON from your vendordep runtime setup (Java/native side) so
teams do not need to run a separate Python state-tracker process.

## Seasonal Workflow

### Step 1: Get Field Dimensions & Rules (End of Championship)

When a new season is announced:
- FRC releases official field specifications (dimensions, materials, positions)
- Game elements are defined (mass, material properties)
- Interaction properties are determined

### Step 2: Create Field Definition JSON

Define the new season's field with all elements:

```json
{
  "year": 2026,
  "game": "EVERGREEN",
  "field_dimensions": {
    "length": 16.54,
    "width": 8.21
    },
    "field_boundary": {
        "shape": "rectangle",
        "vertices": [
            {"x": 0.0, "y": 0.0},
            {"x": 16.54, "y": 0.0},
            {"x": 16.54, "y": 8.21},
            {"x": 0.0, "y": 8.21}
        ],
        "notes": "Use polygon vertices for seasons with angled edges or cut corners."
  },
  "elements": [
    {
      "name": "element_name",
      "type": "goal|platform|frame|zone",
      "pose": {
        "x": 0.0, "y": 0.0, "z": 0.0,
        "roll": 0.0, "pitch": 0.0, "yaw": 0.0
      },
      "dimensions": {
        "length": 1.0,
        "width": 1.0
      },
      "material": "aluminum|steel|wood|plastic",
      "mass": 50.0
    }
  ],
  "game_pieces": [
    {
      "type": "note|ball|cube|cone|ring",
      "initial_count": 5,
      "mass": 0.235,
      "material": "rubber_composite",
      "diameter": 0.375
    }
  ],
  "constraints": {
    "max_robot_size": 1.20,
    "max_robot_mass": 70.0
  }
}
```

### Step 3: Update Field Definition Class

Add seasonal field class to `field_definitions.py`:

```python
class Field2026Definition:
    """EVERGREEN 2026 field definition."""
    
    FIELD_LENGTH = 16.54
    FIELD_WIDTH = 8.21
    
    @staticmethod
    def get_field_definition() -> Dict[str, Any]:
        return {
            "year": 2026,
            "game": "EVERGREEN",
            # ... full field definition
        }
```

Then register in manager:

```python
FieldDefinitionManager.SEASONS[2026] = Field2026Definition
```

### Step 4: Define Material Properties

Create material interaction matrix in `config.py`:

```python
MATERIALS = {
    "material_name": (density_kg_m3, friction_coeff, restitution),
    "aluminum": (2700, 0.3, 0.3),
    "field_special_material": (1500, 0.4, 0.25),
}
```

### Step 5: Create Season Setup Script

```python
from field_definitions import FieldDefinitionManager
import json

def setup_2026_field():
    """Create a runtime-consumable field JSON for 2026 season."""
    
    # Load field definition
    field_def = FieldDefinitionManager.get_field_definition(2026)
    
    with open("field_2026.json", "w") as f:
        json.dump(field_def, f, indent=2)

    return field_def

# In vdep runtime init
field = setup_2026_field()
```

## Key Components

### Runtime Ownership

The vendordep/runtime layer should own:
- Arena state tracking
- NT/log publishing
- Visualization adapters

`cad-import` should only define and validate season field JSON.

### Game Piece Types

Define game piece types in field JSON:

```python
piece = {
    "type": "note",
    "initial_count": 5,
    "mass": 0.235,
    "material": "rubber_composite",
}
```

Supported types:
- `NOTE`: 2024 game piece
- `BALL`: 2025+ game pieces
- `CUBE`/`CONE`/`RING`: Various FRC pieces
- `GAMEPIECE`: Generic

## Material Interactions

Define how materials interact for physics:

```python
material_interactions = {
    ("rubber", "aluminum"): (0.4, 0.3),      # (friction, restitution)
    ("rubber", "wood"): (0.5, 0.25),
    ("aluminum", "aluminum"): (0.3, 0.2),
}

# Apply material of field element
element = FieldElement(
    name="speaker_rim",
    material="aluminum",
    # ...
)

# Game pieces also have material
note = GamePiece(
    id="note_01",
    material="rubber_composite",
    # ...
)
```

## Performance Optimization (High-Frequency Updates)

For smooth 20ms+ updates, follow these guidelines:

### ✓ DO

**Preallocate arrays:**
```python
# Good: Create lists once
robot_poses = [None] * 12
for i, robot in enumerate(active_robots):
    robot_poses[i] = robot.pose
```

**Batch updates:**
```python
# Good: Update with single lock
with arena._lock:
    for team, pose in batch_updates:
        arena.update_robot_pose(team, pose, velocity)
```

**Reuse objects:**
```python
# Good: Reuse allocation
pose = Pose3d(0, 0, 0, 0, 0, 0)
for step in range(1000):
    pose.x = calculate_x()
    # ... reuse same object
```

### ✗ DON'T

**Linked lists:**
```python
# Bad: LinkedList is slow
from collections import deque
pieces = deque()  # Don't use for HF updates
```

**Allocate in loops:**
```python
# Bad: Creates new list each iteration
for step in range(1000):
    poses = []  # Allocate every iteration
    poses.append(robot.pose)
```

**Excessive dictionary lookups:**
```python
# Bad: Lookup every iteration
for _ in range(1000):
    x = config["field"]["length"]  # Expensive
    
# Good: Cache
field_length = config["field"]["length"]
for _ in range(1000):
    x = field_length  # Fast
```

## Testing

### Unit Tests

```python
def test_arena_creation():
    arena = ArenaState(16.54, 8.21)
    assert arena.field_length == 16.54

def test_add_robot():
    arena = ArenaState()
    robot = Robot(team_number=1690, alliance="blue", pose=...)
    assert arena.add_robot(robot)
    assert robot.team_number in arena.robots

def test_thread_safety():
    arena = ArenaState()
    # Concurrent updates should not crash
    # (verified by threading tests)
```

### Validation

```python
# Verify field definition structure
field_def = FieldDefinitionManager.get_field_definition(2024)
assert "year" in field_def
assert "elements" in field_def
assert "game_pieces" in field_def

# Verify arena state
state = arena.get_state_snapshot()
assert "robots" in state
assert "game_pieces" in state
assert "field_elements" in state
```

## Troubleshooting

### Runtime Capacity Error

```
WARNING: Runtime capacity reached for game pieces
```

**Solution:** Increase game-piece capacity in the vendordep runtime config.

```python
self._max_game_pieces = 512  # Increase limit
```

### High Memory Usage

**Issue:** Each update creates new objects

**Solution:** Reuse Pose3d objects, batch updates

```python
# Good: Reuse
pose = Pose3d(0, 0, 0, 0, 0, 0)
for update in updates:
    pose.x = update.x
    arena.update_robot_pose(team, pose, velocity)
```

### Slow Viewer Updates

**Issue:** Visualizer lag in your chosen viewer

**Solution:** 
1. Reduce update frequency (20ms minimum)
2. Batch runtime publishes
3. Reduce number of game pieces being tracked

```python
# Update every 50ms instead of 20ms if needed
if frame_count % (50 / 20) == 0:
    publish_runtime_snapshot()
```

## References

- [Field Definitions](field_definitions.py)
- [Field Setup Examples](field_setup_examples.py)

## Example Timeline: 2026 Season

### April 2026 (Post-Game Announcement)

✓ Get official 2026 field dimensions and CAD  
✓ Review rebuilt field layout and unique elements  
✓ Create JSON field definition  
✓ Add Field2026Definition class with actual specs  
✓ Define material properties for new field elements  

### May 2026 (Pre-Season)

✓ Test field setup with example teams  
✓ Generate optional visualization adapters as needed  
✓ Create season-specific documentation  

### June 2026 (Build Season)

✓ Teams import their robot CAD  
✓ Validate robot CAD in field context  
✓ Run physics simulations with 2026 field  

### Competition Season

✓ Teams use JSim for match prediction on 2026 field  
✓ Live visualization in AdvantageScope  
✓ Real-time physics validation with unique field elements  
