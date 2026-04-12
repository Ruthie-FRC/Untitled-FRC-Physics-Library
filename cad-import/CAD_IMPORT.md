# JSim CAD Import System

A comprehensive framework for importing CAD models into JSim from OnShape exports, with support for multiple accuracy levels and grouped mechanisms.

Based on [Discussion #36](https://github.com/Ruthie-FRC/JSim/discussions/36): Strategic approach to CAD import focusing on grouped mechanisms rather than individual components.

## Overview

The CAD import system enables FRC teams to:

- **Import from OnShape**: Direct glTF conversion from OnShape exports
- **Multiple Accuracy Levels**: HIGH (most accurate), MEDIUM (balanced), LOW (fastest)
- **Smart Material Handling**: Default aluminum if material not specified
- **Grouped Mechanisms**: Focus on exported mechanisms, not every individual component
- **Export Flexibility**: JSON, Java code generation, and extensible format support
- **Graceful Degradation**: Works with imperfect CAD, not a nightmare for teams

## Key Principles

From the discussion with @thenetworkgrinch:

### ✓ DO

- **Export grouped mechanisms** from OnShape (not individual parts)
- **Export in-state**: Export assemblies while mechanisms are in proper position (arms extended, etc.)
- **Use OnShape simulation**: Teams should group mechanisms they've already simulated in OnShape
- **Set reasonable defaults**: Assume aluminum for undefined materials
- **Multiple accuracy tiers**: Let teams choose accuracy vs. performance

### ✗ DON'T

- **Simulate every component**: Only simulate grouped mechanisms
- **Model trivial details**: Skip cosmetic features at lower accuracy levels
- **Require perfect CAD**: Work with real-world team CAD including bad practices
- **Force material definitions**: Handle missing materials gracefully
- **Simulate fasteners at low accuracy**: Skip bolts/screws at lower accuracy levels

## Architecture

```
cad-import/
├── __init__.py           # Main API exports
├── config.py             # Configuration, accuracy levels, materials
├── materials.py          # Material system with intelligent defaults
├── mechanisms.py         # Grouped mechanism extraction and validation
├── importer.py           # CAD import (OnShape glTF, metadata)
├── exporter.py           # Export (JSON, Java code, etc.)
├── examples.py           # Usage examples
├── CAD_IMPORT.md        # This file
└── tests/                # Unit tests (to be added)
```

## Quick Start

### Basic Robot Import

```python
from cad_import import OnShapeCADImporter, AccuracyLevel, ExportFormat, UniversalCADExporter

# Create importer (Medium accuracy by default)
importer = OnShapeCADImporter(AccuracyLevel.MEDIUM)

# Import OnShape exports
importer.import_gltf("robot.gltf")
importer.import_from_onshape_metadata("robot_mechanisms.json")

# Export to JSON
mechanisms = importer.get_mechanisms_to_export()
exporter = UniversalCADExporter()
exporter.export(mechanisms, "robot.json", ExportFormat.JSON)
```

### Field Elements Import

```python
from cad_import import FieldCADImporter

# Field importer uses HIGH accuracy automatically
field_importer = FieldCADImporter(field_year=2024)
field_importer.import_field_definition("field_2024.json")

mechanisms = field_importer.get_mechanisms_to_export()
exporter.export(mechanisms, "field_2024.json")
```

## Accuracy Levels

| Level  | Use Case | Collision Detail | Fasteners | Min Mass | Primitives |
|--------|----------|------------------|-----------|----------|-----------|
| **HIGH** | Exact simulation, lab testing | Full | Yes | 1g | 100 |
| **MEDIUM** | Typical team competition | Full | No | 50g | 50 |
| **LOW** | Fast iteration, development | Basic | No | 100g | 20 |

## Material System

### Built-in Materials

```python
MATERIALS = {
    "aluminum": (2700 kg/m³, μ=0.3, e=0.3),
    "steel": (7850 kg/m³, μ=0.4, e=0.2),
    "titanium": (4500 kg/m³, μ=0.35, e=0.25),
    "carbon_fiber": (1600 kg/m³, μ=0.3, e=0.25),
    # ... more materials
}
```

### Smart Defaults

- **Undefined materials** → Aluminum (most common in FRC)
- **Mass calculation**: `mass = volume * density`
- **Custom materials**: Easy to add via `MaterialSystem.add_custom_material()`

### Example: Custom Materials

```python
material_system = MaterialSystem()
material_system.add_custom_material("team_special_alloy", 2800, 0.32, 0.28)

importer = OnShapeCADImporter()
importer.material_system = material_system
```

## Grouped Mechanism Format

### Expected OnShape Metadata (JSON)

```json
{
  "mechanisms": [
    {
      "name": "shooter",
      "type": "motor_driven",
      "components": [
        {
          "name": "flywheel",
          "material": "aluminum",
          "mass": 1.2,
          "volume": 0.0001,
          "collision_enabled": true,
          "is_fastener": false
        }
      ],
      "joints": [
        {
          "name": "spinner_bearing",
          "type": "revolute",
          "parent": "frame",
          "child": "flywheel",
          "axis": [0, 1, 0],
          "limits": null
        }
      ]
    }
  ]
}
```

### Mechanism Types

- `MOTOR_DRIVEN`: Mechanisms powered by motors
- `LINKED_ASSEMBLY`: Linkages (4-bar, etc.)
- `ROTATING_JOINT`: Single-axis rotations
- `LINEAR_JOINT`: Prismatic/linear actuator
- `FIXED`: Stationary components

## Export Formats

### JSON Export
```python
exporter.export(mechanisms, "robot.json", ExportFormat.JSON)
# Produces structured JSON with all mechanism data
```

### Java Code Generation
```python
exporter.export(mechanisms, "./generated", ExportFormat.CUSTOM, 
                package_name="frc.robot.sim")
# Generates PhysicsBody setup code for JSim
```

### Summary Report
```python
exporter.export_summary_report(mechanisms, "report.md")
# Human-readable summary for verification
```

## Workflow: Team's Perspective

1. **Design in OnShape**
   - Create mechanism assemblies
   - Group correlated components
   - Run OnShape simulation to verify motion

2. **Export from OnShape**
   - Export as glTF binary (.glb) or text (.gltf)
   - Save mechanism definitions as JSON metadata

3. **Import to JSim**
   ```python
   importer = OnShapeCADImporter(AccuracyLevel.MEDIUM)
   importer.import_gltf("robot.gltf")
   importer.import_from_onshape_metadata("mechanisms.json")
   ```

4. **Customize (Optional)**
   - Adjust materials
   - Mark fasteners
   - Change accuracy level

5. **Export for Simulation**
   ```python
   mechanisms = importer.get_mechanisms_to_export()
   exporter.export(mechanisms, "robot_physics.json")
   ```

## Edge Cases & Robustness

### Handling Missing Information

| Issue | Behavior |
|-------|----------|
| No material specified | Use default (aluminum) |
| Zero mass component | Treat as fixed body |
| Missing geometry | Skip collision |
| Unrealistic mass | Log warning, continue |
| No joints | Treat as single rigid body |

### Team CAD Challenges
- **Imperfect assemblies**: Works fine (only groups exported)
- **Missing bolts**: Ignored unless simulating fasteners
- **Nested assemblies**: Only top-level groups exported
- **Complex linkages**: Export while in correct state (extended, etc.)

## Validation & Debugging

```python
# Validate all mechanisms
is_valid, issues = importer.validate_all()
if not is_valid:
    for issue in issues:
        print(f"Issue: {issue}")

# Get detailed summary
summary = importer.get_import_summary()
print(f"Mechanisms: {summary['total_mechanisms']}")
print(f"Components: {summary['total_components']}")

# Export diagnostic report
exporter.export_summary_report(mechanisms, "diagnostics.md")
```

## Performance Considerations

### File Sizes
- **HIGH** accuracy: Larger exports, slower simulation
- **MEDIUM** accuracy: Balanced (recommended default)
- **LOW** accuracy: Smallest exports, fastest simulation

### Optimization Tips
1. Use appropriate accuracy level
2. Disable collision for non-interacting bodies
3. Mark small components as fasteners to skip
4. Limit collision primitives count
5. Remove cosmetic geometry

## Future Enhancements

- [ ] Real GLB binary parsing (currently placeholder)
- [ ] URDF export format
- [ ] Automatic mechanism grouping from OnShape
- [ ] C++ code generation
- [ ] Physics validation tools
- [ ] CAD quality scoring
- [ ] Interactive import GUI

## Troubleshooting

### glTF Import Fails
- Check file exists and format is correct (.gltf or .glb)
- Ensure OnShape export includes all mechanisms
- Try with lower accuracy level

### Missing Mechanisms
- Verify metadata file has correct JSON structure
- Check mechanism names match component definitions
- Ensure root_component exists in components list

### Unrealistic Physics
- Verify material assignments
- Check volume calculations
- Compare with actual robot weight
- Try HIGH accuracy level

### Export Generation Fails
- Ensure output directory exists and is writable
- Check for invalid Java class names in mechanism names
- Verify package name is valid Java package

## Contributing

To add new features:

1. Consider accuracy level impact
2. Add tests to verify edge cases
3. Update documentation
4. Test with real team CAD files

## References

- [Discussion #36: CAD Import Strategy](https://github.com/Ruthie-FRC/JSim/discussions/36)
- [AdvantageScope glTF Conversion](https://docs.advantagescope.org/more-features/custom-assets/gltf-convert)
- [glTF Specification](https://www.khronos.org/gltf/)

## License

Same as JSim (TBD)
