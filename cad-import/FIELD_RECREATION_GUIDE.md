# Seasonal Field Recreation Guide

Complete guide for recreating FRC field definitions and game elements for JSim each season.

Scope note: `cad-import` is responsible for generating and validating season JSON.
Runtime state tracking, NT publishing, and live visualization should be implemented
inside the vendordep/runtime layer.

## CAD Source Format Policy (Onshape)

Use STEP as the source export format from Onshape.

- Export from Onshape as `.step`/`.stp`.
- Do not treat glTF as the source-of-truth CAD artifact.
- If you need to use the current importer path, convert STEP to glTF as a preprocessing step.

This keeps a stable neutral CAD source while preserving compatibility with the current importer.

## Quick Start

For students/teams using JSim vendordep in robot code:
- No separate Python script is required at runtime.
- No separate field JSON file is required for built-in seasons.
- Select a season year from vendordep and run.

### 1. Student Runtime Path (Vendordep Built-In)

```java
import com.fasterxml.jackson.databind.JsonNode;
import jsim.field.FieldDefinitionCatalog;

JsonNode field = FieldDefinitionCatalog.loadFieldNode(2024);
// Use field in runtime setup
```

### 2. Maintainer Path: Load Existing Field Definition

```python
from field_definitions import FieldDefinitionManager

# Get 2024 CRESCENDO field
field_def = FieldDefinitionManager.get_field_definition(2024)
print(f"Game: {field_def['game']}")
print(f"Elements: {len(field_def['elements'])}")
```

### 3. Maintainer Path: Validate Field JSON

```python
dims = field_def["field_dimensions"]
assert "length" in dims and "width" in dims
assert "elements" in field_def
assert "game_pieces" in field_def
```

### 4. Maintainer Path: Export JSON for Runtime Consumption

```python
import json

with open("field_2026.json", "w") as f:
    json.dump(field_def, f, indent=2)
```

### 5. Runtime Consumption

Built-in seasons should be loaded by year from vendordep runtime (no extra
field file handling by students).

Generated JSON is mainly for maintainer workflows, custom events, or pre-release
validation.

## Seasonal Workflow

### Step 1: Get Field Dimensions & Rules (End of Championship)

When a new season is announced:
- FRC releases official field specifications (dimensions, materials, positions)
- Game elements are defined (mass, material properties)
- Interaction properties are determined

### Step 2: Create Field Definition JSON

Define the new season's field with all elements:

For code-level API documentation templates (Doxygen/Javadoc-style) when adding
new season arenas/layouts/game elements, see:

- `mkdocs/docs/season_authoring.md`

`field_dimensions` should be treated as a bounding box. The actual playable
shape is defined by `field_boundary.vertices`, which supports both rectangles
and angled-edge polygons.

```json
{
  "year": 2026,
  "game": "EVERGREEN",
  "field_dimensions": {
    "length": 16.54,
    "width": 8.21
    },
    "field_boundary": {
        "shape": "polygon",
        "vertices": [
            {"x": 0.0, "y": 0.0},
            {"x": 16.54, "y": 0.0},
            {"x": 16.54, "y": 7.20},
            {"x": 15.80, "y": 8.21},
            {"x": 0.74, "y": 8.21},
            {"x": 0.0, "y": 7.20}
        ],
        "notes": "Use 4 vertices for rectangles, or 5+ vertices for angled/cut corners."
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
class Field2026WeldedDefinition:
    """EVERGREEN 2026 field definition."""
    
    # Keep length/width as a bounding box for quick checks.
    # Use field_boundary vertices for angled or non-rectangular edges.
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
FieldDefinitionManager.SEASONS[2026] = Field2026WeldedDefinition
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

### Step 5: Generate Season JSON (Maintainer/CI Step)

```python
from field_definitions import FieldDefinitionManager
import json

def generate_2026_field_json():
    """Create a runtime-consumable field JSON for 2026 season."""
    
    # Load field definition
    field_def = FieldDefinitionManager.get_field_definition(2026)
    
    with open("field_2026.json", "w") as f:
        json.dump(field_def, f, indent=2)

    return field_def

generate_2026_field_json()
```

Teams should consume the generated JSON through vendordep/runtime APIs; this
Python step is for maintainers/CI to produce season files, not for per-team
runtime state tracking.

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

## Runtime Performance Notes

For smooth 20ms+ updates in vendordep/runtime, follow these guidelines:

### ✓ DO

**Preallocate runtime buffers:**
```python
pose7 = [0.0] * (max_bodies * 7)
vel6 = [0.0] * (max_bodies * 6)
```

**Batch publish from runtime snapshot:**
```python
count = runtime.get_body_state13_array(state13)
nt.publish(state13[:count * 13])
```

**Reuse allocations:**
```python
state13 = [0.0] * (max_bodies * 13)
# Reuse `state13` every frame instead of allocating each loop.
```

### ✗ DON'T

**Allocate in loops:**
```python
# Bad: Creates new array every frame
for step in range(1000):
    state13 = [0.0] * (max_bodies * 13)
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
def test_field_boundary_schema():
    field_def = FieldDefinitionManager.get_field_definition(2024)
    assert "field_boundary" in field_def
    assert len(field_def["field_boundary"]["vertices"]) >= 4

def test_polygon_boundary_supported():
    custom = FieldDefinitionManager.ensure_field_boundary({...})
    assert custom["field_boundary"]["shape"] in ["rectangle", "polygon"]
```

### Validation

```python
# Verify field definition structure
field_def = FieldDefinitionManager.get_field_definition(2024)
assert "year" in field_def
assert "elements" in field_def
assert "game_pieces" in field_def

# Verify boundary data exists for runtime consumption
assert "field_boundary" in field_def
assert "vertices" in field_def["field_boundary"]
```

## Troubleshooting

### Runtime Capacity Error

```
WARNING: Runtime capacity reached for game pieces
```

**Solution:** Increase game-piece capacity in the vendordep runtime config.

```python
runtime.max_game_pieces = 512
```

### High Memory Usage

**Issue:** Runtime allocates new arrays each frame

**Solution:** Reuse preallocated state arrays/buffers

```python
state13 = [0.0] * (max_bodies * 13)
# Reuse state13 every loop iteration
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
✓ Add Field2026WeldedDefinition class with actual specs  
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
