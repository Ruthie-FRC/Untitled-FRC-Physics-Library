"""
JSim CAD Import System

Comprehensive CAD import framework for FRC robot and field simulations.
Uses an Onshape STEP-first workflow, with optional STEP-to-glTF preprocessing for the current importer path.
Supports multiple accuracy levels and grouped mechanisms.

Example Usage:

    from cad_import import OnShapeCADImporter, AccuracyLevel, ExportFormat

    # Create importer with medium accuracy
    importer = OnShapeCADImporter(AccuracyLevel.MEDIUM)
    
    # Recommended: export STEP from Onshape, convert to glTF, then import
    # Import CAD from converted glTF (.gltf/.glb)
    importer.import_gltf("robot.gltf")
    
    # Import mechanism metadata
    importer.import_from_onshape_metadata("mechanisms.json")
    
    # Get summary
    summary = importer.get_import_summary()
    print(f"Imported {summary['total_mechanisms']} mechanisms")
    
    # Export to JSON
    mechanisms = importer.get_mechanisms_to_export()
    exporter = UniversalCADExporter()
    exporter.export(mechanisms, "output.json", ExportFormat.JSON)
"""

from .config import (
    AccuracyLevel,
    IMPORT_ACCURACY_PROFILES,
    ACCURACY_CONFIGS,
    MATERIALS,
    DEFAULT_MATERIAL,
    ModelValidationRules,
    ExportFormat,
)

from .materials import MaterialSystem

from .mechanisms import (
    MechanismType,
    Component,
    Joint,
    GroupedMechanism,
    MechanismExtractor,
)

from .importer import (
    OnShapeCADImporter,
    FieldCADImporter,
)

from .exporter import (
    CADExporter,
    UniversalCADExporter,
)

from .field_definitions import (
    FieldDefinitionManager,
    Field2024Definition,
    Field2025WeldedDefinition,
    Field2025AndyMarkDefinition,
    Field2026WeldedDefinition,
    Field2026AndyMarkDefinition,
)

def get_apriltag_layout(year: int, variant: str = None):
    """Convenience helper to fetch normalized AprilTag layout by year."""
    return FieldDefinitionManager.get_apriltag_layout(year, variant=variant)

__version__ = "0.2.0"
__author__ = "JSim"

__all__ = [
    # Configuration
    "AccuracyLevel",
    "IMPORT_ACCURACY_PROFILES",
    "ACCURACY_CONFIGS",
    "MATERIALS",
    "DEFAULT_MATERIAL",
    "ModelValidationRules",
    "ExportFormat",
    # Materials
    "MaterialSystem",
    # Mechanisms
    "MechanismType",
    "Component",
    "Joint",
    "GroupedMechanism",
    "MechanismExtractor",
    # Importer
    "OnShapeCADImporter",
    "FieldCADImporter",
    # Exporter
    "CADExporter",
    "UniversalCADExporter",
    # Field definitions
    "FieldDefinitionManager",
    "Field2024Definition",
    "Field2025WeldedDefinition",
    "Field2025AndyMarkDefinition",
    "Field2026WeldedDefinition",
    "Field2026AndyMarkDefinition",
    "get_apriltag_layout",
]
