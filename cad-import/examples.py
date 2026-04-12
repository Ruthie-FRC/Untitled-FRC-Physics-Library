"""
Examples of using the JSim CAD Import system.

These examples demonstrate the workflow for importing CAD from OnShape
and exporting for simulation in JSim.
"""

from cad_import import (
    OnShapeCADImporter,
    FieldCADImporter,
    AccuracyLevel,
    ExportFormat,
    UniversalCADExporter,
    MaterialSystem,
)
import json


def example_robot_import():
    """Example: Import robot CAD from OnShape export."""
    print("Example: Robot CAD Import")
    print("-" * 50)
    
    # Create importer with medium accuracy (balanced)
    importer = OnShapeCADImporter(AccuracyLevel.MEDIUM)
    
    # Import glTF exported from OnShape
    if importer.import_gltf("robot_export.gltf"):
        print("✓ Successfully imported glTF file")
    else:
        print("✗ Failed to import glTF file")
        return
    
    # Import OnShape mechanism definitions
    if importer.import_from_onshape_metadata("robot_mechanisms.json"):
        print("✓ Successfully imported mechanism metadata")
    else:
        print("✗ Failed to import mechanism metadata")
    
    # Display import summary
    summary = importer.get_import_summary()
    print(f"\nImport Summary:")
    print(f"  Total mechanisms: {summary['total_mechanisms']}")
    print(f"  Mechanisms to export: {summary['mechanisms_to_export']}")
    print(f"  Mechanism types: {summary['mechanism_types']}")
    print(f"  Total components: {summary['total_components']}")
    print(f"  Total joints: {summary['total_joints']}")
    
    # Validate imported data
    all_valid, issues = importer.validate_all()
    if all_valid:
        print("✓ All mechanisms valid")
    else:
        print(f"⚠ Validation issues: {len(issues)}")
        for issue in issues:
            print(f"  - {issue}")
    
    # Export to JSON
    mechanisms = importer.get_mechanisms_to_export()
    exporter = UniversalCADExporter()
    
    if exporter.export(mechanisms, "robot_jsim.json", ExportFormat.JSON):
        print("\n✓ Successfully exported to robot_jsim.json")
    
    # Export to Java codegen
    if exporter.export(mechanisms, "./generated_robot", ExportFormat.CUSTOM, package_name="frc.robot.sim"):
        print("✓ Successfully exported to Java code")
    
    return mechanisms


def example_accuracy_levels():
    """Example: Compare different accuracy levels."""
    print("\n\nExample: Accuracy Levels Comparison")
    print("-" * 50)
    
    # Same CAD, different accuracy levels
    gltf_file = "robot_export.gltf"
    
    for accuracy in [AccuracyLevel.HIGH, AccuracyLevel.MEDIUM, AccuracyLevel.LOW]:
        print(f"\nProcessing with {accuracy.value.upper()} accuracy:")
        
        importer = OnShapeCADImporter(accuracy)
        if importer.import_gltf(gltf_file):
            mechanisms = importer.get_mechanisms_to_export()
            total_components = sum(len(m.components) for m in mechanisms)
            print(f"  Mechanisms: {len(mechanisms)}")
            print(f"  Components: {total_components}")


def example_material_customization():
    """Example: Customize materials for imported components."""
    print("\n\nExample: Material Customization")
    print("-" * 50)
    
    importer = OnShapeCADImporter(AccuracyLevel.MEDIUM)
    
    # Define custom materials
    custom_materials = {
        "custom_aluminum": (2700, 0.3, 0.3),
        "heavy_steel": (8000, 0.4, 0.2),
    }
    
    importer.material_system = MaterialSystem(custom_materials)
    print("✓ Added custom materials")
    
    # Import CAD
    importer.import_gltf("robot_export.gltf")
    importer.import_from_onshape_metadata("robot_mechanisms.json")
    
    # Set specific materials for components
    importer.set_component_material("frame", "custom_aluminum")
    importer.set_component_material("base", "heavy_steel")
    print("✓ Set component materials")
    
    # Mark fasteners to handle appropriately
    importer.mark_fasteners(["bolt", "screw", "nut", "washer"])
    print("✓ Marked fastener components")
    
    # Export
    exporter = UniversalCADExporter(importer.material_system)
    mechanisms = importer.get_mechanisms_to_export()
    exporter.export(mechanisms, "robot_custom_materials.json", ExportFormat.JSON)
    print("✓ Exported with custom materials")


def example_field_import():
    """Example: Import FRC field elements."""
    print("\n\nExample: FRC Field Import")
    print("-" * 50)
    
    # Create field-specific importer (automatically uses HIGH accuracy)
    field_importer = FieldCADImporter(field_year=2024)
    
    # Import field definition
    if field_importer.import_field_definition("field_2024.json"):
        print("✓ Successfully imported field definition")
    else:
        print("✗ Failed to import field")
        return
    
    # Display field summary
    summary = field_importer.get_import_summary()
    print(f"\nField Summary:")
    print(f"  Year: {summary['metadata'].get('field_year', 'Unknown')}")
    print(f"  Elements: {summary['total_mechanisms']}")
    print(f"  Components: {summary['total_components']}")
    
    # Export field to JSON
    mechanisms = field_importer.get_mechanisms_to_export()
    exporter = UniversalCADExporter()
    exporter.export(mechanisms, "field_2024_jsim.json", ExportFormat.JSON)
    print("\n✓ Exported field to field_2024_jsim.json")


def example_batch_import():
    """Example: Batch import multiple robot CAD files."""
    print("\n\nExample: Batch Import")
    print("-" * 50)
    
    robot_files = [
        ("team1690.gltf", "team1690_mechanisms.json"),
        ("team2910.gltf", "team2910_mechanisms.json"),
        ("team1234.gltf", "team1234_mechanisms.json"),
    ]
    
    all_exports = {}
    
    for gltf_file, metadata_file in robot_files:
        print(f"\nImporting {gltf_file}...")
        
        importer = OnShapeCADImporter(AccuracyLevel.MEDIUM)
        
        try:
            importer.import_gltf(gltf_file)
            importer.import_from_onshape_metadata(metadata_file)
            
            mechanisms = importer.get_mechanisms_to_export()
            summary = importer.get_import_summary()
            
            all_exports[gltf_file] = {
                "mechanisms": mechanisms,
                "summary": summary
            }
            
            print(f"  ✓ Imported {summary['total_mechanisms']} mechanisms")
            
        except Exception as e:
            print(f"  ✗ Failed: {e}")
    
    # Summary of all imports
    print(f"\n\nBatch Import Summary:")
    print(f"Successfully imported: {len(all_exports)}/{len(robot_files)} files")
    for name, data in all_exports.items():
        print(f"  {name}: {data['summary']['total_mechanisms']} mechanisms")


def example_metadata_format():
    """Example: OnShape metadata format for mechanism definitions.
    
    This shows the expected JSON format for OnShape mechanism definitions.
    """
    print("\n\nExample: OnShape Metadata Format")
    print("-" * 50)
    
    # Example metadata structure
    metadata = {
        "mechanisms": [
            {
                "name": "drivetrain",
                "type": "motor_driven",
                "components": [
                    {
                        "name": "left_side_frame",
                        "material": "aluminum",
                        "mass": 2.5,
                        "volume": 0.001,
                        "collision_enabled": True,
                        "is_fastener": False,
                    },
                    {
                        "name": "left_motor_mount",
                        "material": "aluminum",
                        "mass": 0.3,
                        "volume": 0.0001,
                        "collision_enabled": True,
                    },
                ],
                "joints": [
                    {
                        "name": "left_wheel_axle",
                        "type": "revolute",
                        "parent": "left_side_frame",
                        "child": "left_motor_mount",
                        "axis": [0, 1, 0],
                        "limits": None,
                    }
                ],
            },
            {
                "name": "shooter",
                "type": "motor_driven",
                "components": [
                    {
                        "name": "shooter_wheel",
                        "material": "aluminum",
                        "mass": 1.2,
                        "volume": 0.00005,
                    }
                ],
                "joints": [],
            },
        ]
    }
    
    print("Expected OnShape metadata format:")
    print(json.dumps(metadata, indent=2))
    
    # Save example
    with open("example_mechanisms.json", "w") as f:
        json.dump(metadata, f, indent=2)
    
    print("\n✓ Saved example_mechanisms.json")


if __name__ == "__main__":
    print("JSim CAD Import Examples")
    print("=" * 50)
    
    # Run examples (with error handling)
    try:
        example_metadata_format()
        # example_robot_import()
        # example_accuracy_levels()
        # example_material_customization()
        # example_field_import()
        # example_batch_import()
        
        print("\n" + "=" * 50)
        print("Examples completed!")
        
    except Exception as e:
        print(f"\n✗ Example failed with error: {e}")
        import traceback
        traceback.print_exc()
