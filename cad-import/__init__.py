"""
JSim CAD Import System

Comprehensive CAD import framework for FRC robot and field simulations.
Supports OnShape exports (glTF), multiple accuracy levels, and grouped mechanisms.

From Discussion #36: Focus on grouped mechanisms, not individual components.

Example Usage:

    from cad_import import OnShapeCADImporter, AccuracyLevel, ExportFormat

    # Create importer with medium accuracy
    importer = OnShapeCADImporter(AccuracyLevel.MEDIUM)
    
    # Import CAD from OnShape glTF export
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
    AccuracyConfig,
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

__version__ = "0.1.0"
__author__ = "Ruthie-FRC"

__all__ = [
    # Configuration
    "AccuracyLevel",
    "AccuracyConfig",
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
]
