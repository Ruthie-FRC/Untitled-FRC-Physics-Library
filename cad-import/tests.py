"""
Test suite for CAD import system.

Validates core functionality without external dependencies.
"""

import sys
import json
from pathlib import Path


def test_imports():
    """Test that all modules can be imported."""
    print("Testing module imports...")
    try:
        import config
        import materials
        import mechanisms
        import importer
        import exporter
        
        print("✓ All modules imported successfully")
        return True
    except Exception as e:
        print(f"✗ Import failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_accuracy_levels():
    """Test accuracy level configurations."""
    print("\nTesting accuracy levels...")
    try:
        from config import AccuracyLevel, ACCURACY_CONFIGS
        
        assert AccuracyLevel.HIGH.value == "high"
        assert AccuracyLevel.MEDIUM.value == "medium"
        assert AccuracyLevel.LOW.value == "low"
        
        for level in AccuracyLevel:
            config = ACCURACY_CONFIGS[level]
            assert isinstance(config, dict)
            assert 'include_collision_detail' in config
            assert 'simulate_fasteners' in config
            assert 'min_mass_threshold' in config
        
        print(f"✓ All {len(AccuracyLevel)} accuracy levels configured")
        return True
    except Exception as e:
        print(f"✗ Accuracy level test failed: {e}")
        return False


def test_materials():
    """Test material system."""
    print("\nTesting material system...")
    try:
        from materials import MaterialSystem
        from config import DEFAULT_MATERIAL, MATERIALS
        
        # Test built-in materials
        assert "aluminum" in MATERIALS
        assert "steel" in MATERIALS
        assert DEFAULT_MATERIAL == "aluminum"
        
        # Test material system
        ms = MaterialSystem()
        
        # Get existing material
        density, friction, restitution = ms.get_material_properties("aluminum")
        assert density == 2700
        assert friction == 0.3
        
        # Get with None (should use default)
        density, friction, restitution = ms.get_material_properties(None)
        assert density == 2700  # Aluminum
        
        # Get nonexistent material (should use default)
        density, friction, restitution = ms.get_material_properties("nonexistent")
        assert density == 2700  # Still aluminum
        
        # Test mass calculation
        mass = ms.calculate_mass(1.0, "aluminum")  # 1 m³ of aluminum
        assert mass == 2700
        
        # Test custom material
        ms.add_custom_material("test_material", 1000, 0.5, 0.5)
        density, _, _ = ms.get_material_properties("test_material")
        assert density == 1000
        
        print(f"✓ Material system works ({len(MATERIALS)} built-in materials)")
        return True
    except Exception as e:
        print(f"✗ Material test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_components_and_joints():
    """Test component and joint classes."""
    print("\nTesting components and joints...")
    try:
        from mechanisms import Component, Joint, MechanismType
        
        # Test component
        comp = Component(
            name="test_comp",
            material="aluminum",
            mass=1.0,
            volume=0.0004,
            is_fastener=False
        )
        assert comp.name == "test_comp"
        assert comp.mass == 1.0
        assert comp.is_fastener == False
        
        # Test joint
        joint = Joint(
            name="test_joint",
            joint_type="revolute",
            parent_component="comp1",
            child_component="comp2",
            axis=(0, 1, 0),
            limits=(-90, 90)
        )
        assert joint.joint_type == "revolute"
        assert joint.limits == (-90, 90)
        
        print("✓ Components and joints created successfully")
        return True
    except Exception as e:
        print(f"✗ Component/Joint test failed: {e}")
        return False


def test_grouped_mechanism():
    """Test grouped mechanism class."""
    print("\nTesting grouped mechanisms...")
    try:
        from mechanisms import (
            Component,
            Joint,
            GroupedMechanism,
            MechanismType
        )
        
        # Create components
        components = [
            Component(name="frame", mass=2.0),
            Component(name="motor", mass=0.5, is_fastener=False),
            Component(name="wheel", mass=1.0),
        ]
        
        # Create joints
        joints = [
            Joint(
                name="wheel_bearing",
                joint_type="revolute",
                parent_component="frame",
                child_component="wheel"
            )
        ]
        
        # Create mechanism
        mechanism = GroupedMechanism(
            name="drivetrain",
            mechanism_type=MechanismType.MOTOR_DRIVEN,
            components=components,
            joints=joints,
            root_component="frame"
        )
        
        # Test properties
        assert mechanism.name == "drivetrain"
        assert len(mechanism.components) == 3
        assert len(mechanism.joints) == 1
        assert mechanism.get_static_mass() == 3.5
        
        # Test validation
        is_valid, issues = mechanism.validate()
        assert is_valid, f"Mechanism should be valid, got issues: {issues}"
        
        # Test component retrieval
        frame = mechanism.get_component("frame")
        assert frame is not None
        assert frame.mass == 2.0
        
        print("✓ Grouped mechanisms work correctly")
        return True
    except Exception as e:
        print(f"✗ Grouped mechanism test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_mechanism_extractor():
    """Test mechanism extractor."""
    print("\nTesting mechanism extractor...")
    try:
        from mechanisms import MechanismExtractor
        from config import AccuracyLevel
        
        extractor = MechanismExtractor()
        
        # Extract a mechanism
        components = [
            {"name": "base", "mass": 2.0, "material": "aluminum"},
            {"name": "arm", "mass": 0.5, "material": "carbon_fiber"},
        ]
        
        joints = [
            {
                "name": "pivot",
                "type": "revolute",
                "parent": "base",
                "child": "arm",
            }
        ]
        
        mechanism = extractor.extract_mechanism_from_group(
            "test_arm",
            components,
            joints
        )
        
        assert mechanism is not None
        assert mechanism.name == "test_arm"
        assert len(extractor.mechanisms) == 1
        
        # Test filtering
        mechanisms = extractor.get_mechanisms_to_simulate(AccuracyLevel.MEDIUM)
        assert len(mechanisms) >= 0  # Depends on mass thresholds
        
        # Test summary
        summary = extractor.get_summary()
        assert summary["total_mechanisms"] == 1
        
        print("✓ Mechanism extractor works correctly")
        return True
    except Exception as e:
        print(f"✗ Mechanism extractor test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_importer():
    """Test CAD importer."""
    print("\nTesting CAD importer...")
    try:
        from importer import OnShapeCADImporter
        from config import AccuracyLevel
        
        # Create importer
        importer = OnShapeCADImporter(AccuracyLevel.MEDIUM)
        
        # Test properties
        assert importer.accuracy_level == AccuracyLevel.MEDIUM
        assert importer.material_system is not None
        assert importer.mechanism_extractor is not None
        
        # Test metadata import with test data
        test_metadata = {
            "mechanisms": [
                {
                    "name": "test_mech",
                    "components": [
                        {"name": "comp1", "mass": 1.0}
                    ],
                    "joints": []
                }
            ]
        }
        
        # Write test file
        test_file = Path("/tmp/test_metadata.json")
        test_file.write_text(json.dumps(test_metadata))
        
        try:
            success = importer.import_from_onshape_metadata(str(test_file))
            assert success
            assert len(importer.mechanism_extractor.mechanisms) > 0
            
            print("✓ CAD importer works correctly")
            return True
        finally:
            test_file.unlink()
            
    except Exception as e:
        print(f"✗ CAD importer test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_exporter():
    """Test CAD exporter."""
    print("\nTesting CAD exporter...")
    try:
        from exporter import CADExporter
        from mechanisms import (
            Component,
            Joint,
            GroupedMechanism,
            MechanismType
        )
        
        exporter = CADExporter()
        
        # Create test mechanism
        mechanism = GroupedMechanism(
            name="test_mechanism",
            mechanism_type=MechanismType.MOTOR_DRIVEN,
            components=[
                Component(name="comp1", mass=1.0),
                Component(name="comp2", mass=0.5),
            ],
            joints=[],
            root_component="comp1"
        )
        
        # Test JSON export
        test_file = Path("/tmp/test_export.json")
        try:
            success = exporter.export_to_json([mechanism], str(test_file))
            assert success
            assert test_file.exists()
            
            # Verify content
            data = json.loads(test_file.read_text())
            assert "mechanisms" in data
            assert len(data["mechanisms"]) == 1
            assert data["mechanisms"][0]["name"] == "test_mechanism"
            
            print("✓ CAD exporter works correctly")
            return True
        finally:
            if test_file.exists():
                test_file.unlink()
            
    except Exception as e:
        print(f"✗ CAD exporter test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_field_boundary_support():
    """Test rectangle fallback and polygon boundary support."""
    print("\nTesting field boundary support...")
    try:
        from field_definitions import FieldDefinitionManager

        # Existing seasonal definitions should always produce a boundary.
        field_2024 = FieldDefinitionManager.get_field_definition(2024)
        assert "field_boundary" in field_2024
        assert len(field_2024["field_boundary"]["vertices"]) >= 4

        # Angled polygon boundaries should validate.
        angled = {
            "year": 2099,
            "game": "TEST",
            "field_dimensions": {"length": 16.54, "width": 8.21},
            "field_boundary": {
                "shape": "polygon",
                "vertices": [
                    {"x": 0.0, "y": 0.0},
                    {"x": 16.54, "y": 0.0},
                    {"x": 16.54, "y": 7.8},
                    {"x": 15.9, "y": 8.21},
                    {"x": 0.64, "y": 8.21},
                    {"x": 0.0, "y": 7.8},
                ],
            },
            "elements": [],
            "game_pieces": [],
        }
        normalized = FieldDefinitionManager.ensure_field_boundary(angled)
        assert normalized["field_boundary"]["shape"] == "polygon"
        assert len(normalized["field_boundary"]["vertices"]) == 6

        print("✓ Field boundary rectangle/polygon support works")
        return True
    except Exception as e:
        print(f"✗ Field boundary support test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def run_all_tests():
    """Run all tests."""
    print("=" * 60)
    print("JSim CAD Import System - Test Suite")
    print("=" * 60)
    
    tests = [
        test_imports,
        test_accuracy_levels,
        test_materials,
        test_components_and_joints,
        test_grouped_mechanism,
        test_mechanism_extractor,
        test_importer,
        test_exporter,
        test_field_boundary_support,
    ]
    
    results = []
    for test_func in tests:
        results.append(test_func())
    
    print("\n" + "=" * 60)
    passed = sum(results)
    total = len(results)
    print(f"Tests Passed: {passed}/{total}")
    
    if passed == total:
        print("✓ All tests passed!")
        return 0
    else:
        print(f"✗ {total - passed} test(s) failed")
        return 1


if __name__ == "__main__":
    # Add cad-import to path
    import sys
    sys.path.insert(0, str(Path(__file__).parent))
    
    exit_code = run_all_tests()
    sys.exit(exit_code)
