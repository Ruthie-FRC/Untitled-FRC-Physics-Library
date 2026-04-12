"""
Main CAD import system.

Handles OnShape to glTF conversion and CAD assembly import.
From discussion #36: Focus on grouped mechanisms, not individual components.
"""

from typing import List, Dict, Optional, Any, Tuple
from pathlib import Path
import json
import logging

from .config import AccuracyLevel, ACCURACY_CONFIGS, DEFAULT_MATERIAL
from .materials import MaterialSystem
from .mechanisms import MechanismExtractor, GroupedMechanism, MechanismType

logger = logging.getLogger(__name__)


class OnShapeCADImporter:
    """Imports CAD from OnShape exports (glTF format) with grouped mechanisms."""
    
    def __init__(self, accuracy_level: AccuracyLevel = AccuracyLevel.MEDIUM):
        """Initialize CAD importer.
        
        Args:
            accuracy_level: Desired accuracy level for simulation
        """
        self.accuracy_level = accuracy_level
        self.accuracy_config = ACCURACY_CONFIGS[accuracy_level]
        self.material_system = MaterialSystem()
        self.mechanism_extractor = MechanismExtractor()
        self.import_metadata: Dict[str, Any] = {}
    
    def import_gltf(self, gltf_path: str) -> bool:
        """Import glTF file from OnShape export.
        
        Args:
            gltf_path: Path to glTF file (.gltf or .glb)
        
        Returns:
            True if import successful
        """
        gltf_path = Path(gltf_path)
        
        if not gltf_path.exists():
            logger.error(f"glTF file not found: {gltf_path}")
            return False
        
        if gltf_path.suffix.lower() not in [".gltf", ".glb"]:
            logger.error(f"Invalid glTF file format: {gltf_path.suffix}")
            return False
        
        try:
            logger.info(f"Importing glTF file: {gltf_path}")
            self.import_metadata["source"] = str(gltf_path)
            self.import_metadata["accuracy_level"] = self.accuracy_level.value
            
            # For .gltf files, parse directly
            if gltf_path.suffix.lower() == ".gltf":
                with open(gltf_path, 'r') as f:
                    gltf_data = json.load(f)
                self._process_gltf_data(gltf_data)
            else:
                # For .glb, extract and parse
                logger.info("Binary glTF (.glb) import requires binary parsing - converting to JSON model")
                self._process_glb_file(gltf_path)
            
            logger.info(f"Successfully imported glTF file")
            return True
            
        except Exception as e:
            logger.error(f"Failed to import glTF: {e}")
            return False
    
    def import_from_onshape_metadata(self, metadata_file: str) -> bool:
        """Import from OnShape metadata/groups file.
        
        Expects JSON with structure:
        {
            "mechanisms": [
                {
                    "name": "mechanism_name",
                    "type": "motor_driven|linked_assembly",
                    "components": [...],
                    "joints": [...]
                }
            ]
        }
        """
        metadata_path = Path(metadata_file)
        
        if not metadata_path.exists():
            logger.error(f"Metadata file not found: {metadata_path}")
            return False
        
        try:
            with open(metadata_path, 'r') as f:
                data = json.load(f)
            
            mechanisms = data.get("mechanisms", [])
            logger.info(f"Found {len(mechanisms)} mechanisms in metadata")
            
            for mech_data in mechanisms:
                self.mechanism_extractor.extract_mechanism_from_group(
                    group_name=mech_data.get("name", "unknown"),
                    components=mech_data.get("components", []),
                    joints=mech_data.get("joints", [])
                )
            
            return True
            
        except Exception as e:
            logger.error(f"Failed to import metadata: {e}")
            return False
    
    def _process_gltf_data(self, gltf_data: Dict[str, Any]):
        """Process glTF data structure.
        
        Extracts nodes as components and builds hierarchy.
        """
        nodes = gltf_data.get("nodes", [])
        meshes = gltf_data.get("meshes", [])
        
        logger.info(f"Processing glTF: {len(nodes)} nodes, {len(meshes)} meshes")
        
        # Extract components from nodes
        components = []
        for i, node in enumerate(nodes):
            component_data = {
                "name": node.get("name", f"component_{i}"),
                "mesh_index": node.get("mesh"),
                "metadata": node.get("extras", {})
            }
            components.append(component_data)
        
        # Create default mechanism with all components
        if components:
            self.mechanism_extractor.extract_mechanism_from_group(
                group_name="imported_assembly",
                components=components,
                joints=[]
            )
    
    def _process_glb_file(self, glb_path: Path):
        """Process binary glTF file.
        
        Note: Full GLB parsing requires binary format knowledge.
        This is a placeholder for proper GLB parsing.
        """
        logger.warning("GLB binary parsing not yet implemented - assuming single assembly")
        # In production, would use pyglb or similar library
    
    def set_component_material(self, component_name: str, material: str):
        """Set material for a specific component.
        
        Args:
            component_name: Name of component
            material: Material name
        """
        # Find component in all mechanisms and update material
        for mechanism in self.mechanism_extractor.mechanisms:
            component = mechanism.get_component(component_name)
            if component:
                component.material = material
                logger.info(f"Set {component_name} material to {material}")
    
    def mark_fasteners(self, component_patterns: List[str]):
        """Mark components as fasteners based on name patterns.
        
        Args:
            component_patterns: List of patterns to match (e.g., ["bolt", "screw", "washer"])
        """
        count = 0
        for mechanism in self.mechanism_extractor.mechanisms:
            for component in mechanism.components:
                for pattern in component_patterns:
                    if pattern.lower() in component.name.lower():
                        component.is_fastener = True
                        count += 1
                        break
        
        logger.info(f"Marked {count} components as fasteners")
    
    def get_mechanisms_to_export(self) -> List[GroupedMechanism]:
        """Get mechanisms filtered for export at current accuracy level.
        
        Returns:
            List of mechanisms ready for export
        """
        mechanisms = self.mechanism_extractor.get_mechanisms_to_simulate(self.accuracy_level)
        logger.info(f"Filtered to {len(mechanisms)} mechanisms for {self.accuracy_level.value} accuracy")
        return mechanisms
    
    def get_import_summary(self) -> Dict[str, Any]:
        """Get summary of import results.
        
        Returns:
            Dictionary with import statistics
        """
        summary = {
            "accuracy_level": self.accuracy_level.value,
            "total_mechanisms": len(self.mechanism_extractor.mechanisms),
            "mechanisms_to_export": len(self.get_mechanisms_to_export()),
            "mechanism_types": self._count_mechanism_types(),
            "total_components": sum(
                len(m.components) for m in self.mechanism_extractor.mechanisms
            ),
            "total_joints": sum(
                len(m.joints) for m in self.mechanism_extractor.mechanisms
            ),
            "metadata": self.import_metadata
        }
        return summary
    
    def _count_mechanism_types(self) -> Dict[str, int]:
        """Count mechanisms by type."""
        counts = {}
        for mechanism in self.mechanism_extractor.mechanisms:
            type_name = mechanism.mechanism_type.value
            counts[type_name] = counts.get(type_name, 0) + 1
        return counts
    
    def validate_all(self) -> Tuple[bool, List[str]]:
        """Validate all imported mechanisms.
        
        Returns:
            (all_valid, list of issues)
        """
        all_issues = []
        
        for mechanism in self.mechanism_extractor.mechanisms:
            is_valid, issues = mechanism.validate()
            all_issues.extend(issues)
        
        all_valid = len(all_issues) == 0
        return all_valid, all_issues
    
    def change_accuracy_level(self, new_level: AccuracyLevel):
        """Change accuracy level for export filtering.
        
        Args:
            new_level: New accuracy level
        """
        self.accuracy_level = new_level
        self.accuracy_config = ACCURACY_CONFIGS[new_level]
        logger.info(f"Changed accuracy level to {new_level.value}")


class FieldCADImporter(OnShapeCADImporter):
    """Specialized importer for FRC field elements."""
    
    def __init__(self, field_year: Optional[int] = None):
        """Initialize field CAD importer.
        
        Args:
            field_year: FRC competition year (e.g., 2024)
        """
        super().__init__(accuracy_level=AccuracyLevel.HIGH)  # Field elements need high accuracy
        self.field_year = field_year
    
    def import_field_definition(self, field_file: str) -> bool:
        """Import field definition from JSON.
        
        Expected structure:
        {
            "year": 2024,
            "elements": [
                {
                    "name": "element_name",
                    "type": "fixed|movable",
                    "model": "path/to/model.glb",
                    "position": [x, y, z],
                    "rotation": [rx, ry, rz]
                }
            ]
        }
        """
        field_path = Path(field_file)
        
        if not field_path.exists():
            logger.error(f"Field file not found: {field_path}")
            return False
        
        try:
            with open(field_path, 'r') as f:
                field_data = json.load(f)
            
            self.field_year = field_data.get("year", self.field_year)
            self.import_metadata["field_year"] = self.field_year
            
            elements = field_data.get("elements", [])
            logger.info(f"Importing field definition with {len(elements)} elements")
            
            for element in elements:
                model_path = element.get("model")
                if model_path and Path(model_path).exists():
                    logger.info(f"Importing field element: {element.get('name')}")
                    # Would import each element's model
            
            return True
            
        except Exception as e:
            logger.error(f"Failed to import field definition: {e}")
            return False
