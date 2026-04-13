"""
CAD export system for JSON and code generation.

Exports grouped mechanisms to JSON, CodeGen, or other formats.
From discussion #36: Field Exporter to JSON/CodeGen.
"""

from typing import List, Dict, Any, Optional
from pathlib import Path
import json
import logging

from .config import ExportFormat, DEFAULT_MATERIAL
from .mechanisms import GroupedMechanism, Component, Joint
from .materials import MaterialSystem

logger = logging.getLogger(__name__)


class CADExporter:
    """Exports CAD mechanisms to various formats."""
    
    def __init__(self, material_system: Optional[MaterialSystem] = None):
        """Initialize CAD exporter.
        
        Args:
            material_system: Material system for property lookups
        """
        self.material_system = material_system or MaterialSystem()
    
    def export_to_json(
        self,
        mechanisms: List[GroupedMechanism],
        output_path: str,
        include_geometry: bool = False
    ) -> bool:
        """Export mechanisms to JSON format.
        
        Args:
            mechanisms: List of mechanisms to export
            output_path: Path to output JSON file
            include_geometry: Whether to include detailed geometry data
        
        Returns:
            True if export successful
        """
        output_path = Path(output_path)
        
        try:
            export_data = {
                "version": "1.0",
                "format": "jsim_cad",
                "mechanisms": []
            }
            
            for mechanism in mechanisms:
                mech_json = self._mechanism_to_json(mechanism, include_geometry)
                export_data["mechanisms"].append(mech_json)
            
            # Create parent directory if needed
            output_path.parent.mkdir(parents=True, exist_ok=True)
            
            with open(output_path, 'w') as f:
                json.dump(export_data, f, indent=2)
            
            logger.info(f"Exported {len(mechanisms)} mechanisms to JSON: {output_path}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to export JSON: {e}")
            return False
    
    def _mechanism_to_json(
        self,
        mechanism: GroupedMechanism,
        include_geometry: bool
    ) -> Dict[str, Any]:
        """Convert mechanism to JSON representation."""
        mech_json = {
            "name": mechanism.name,
            "type": mechanism.mechanism_type.value,
            "root_component": mechanism.root_component,
            "components": [],
            "joints": [],
            "metadata": mechanism.metadata
        }
        
        for component in mechanism.components:
            comp_json = {
                "name": component.name,
                "mass": component.mass,
                "volume": component.volume,
                "material": component.material or DEFAULT_MATERIAL,
                "collision_enabled": component.collision_enabled,
                "is_fastener": component.is_fastener
            }
            
            if component.part_number:
                comp_json["part_number"] = component.part_number
            
            if include_geometry and component.metadata.get("geometry"):
                comp_json["geometry"] = component.metadata["geometry"]
            
            mech_json["components"].append(comp_json)
        
        for joint in mechanism.joints:
            joint_json = {
                "name": joint.name,
                "type": joint.joint_type,
                "parent": joint.parent_component,
                "child": joint.child_component,
                "axis": joint.axis
            }
            
            if joint.limits:
                joint_json["limits"] = joint.limits
            
            mech_json["joints"].append(joint_json)
        
        return mech_json
    
    def export_to_java_codegen(
        self,
        mechanisms: List[GroupedMechanism],
        output_path: str,
        package_name: str = "frc.robot.sim"
    ) -> bool:
        """Export mechanisms to Java code generation format.
        
        Generates Java classes for physics bodies and mechanisms.
        
        Args:
            mechanisms: List of mechanisms to export
            output_path: Path to output directory
            package_name: Java package name for generated classes
        
        Returns:
            True if export successful
        """
        output_dir = Path(output_path)
        output_dir.mkdir(parents=True, exist_ok=True)
        
        try:
            # Generate PhysicsWorld setup file
            world_setup = self._generate_java_world_setup(mechanisms, package_name)
            world_file = output_dir / "GeneratedCADWorld.java"
            world_file.write_text(world_setup)
            logger.info(f"Generated Java world setup: {world_file}")
            
            # Generate individual mechanism classes
            for mechanism in mechanisms:
                mech_class = self._generate_java_mechanism_class(mechanism, package_name)
                class_file = output_dir / f"Generated{mechanism.name.replace('-', '_')}.java"
                class_file.write_text(mech_class)
                logger.info(f"Generated Java mechanism class: {class_file}")
            
            return True
            
        except Exception as e:
            logger.error(f"Failed to export Java codegen: {e}")
            return False
    
    def _generate_java_world_setup(
        self,
        mechanisms: List[GroupedMechanism],
        package_name: str
    ) -> str:
        """Generate Java code for PhysicsWorld setup."""
        code = f"""package {package_name};

import frc.robot.sim.*;

/**
 * Auto-generated CAD import for JSim PhysicsWorld.
 * DO NOT EDIT - regenerate from CAD import.
 */
public class GeneratedCADWorld {{
    
    /**
     * Setup physics world with imported mechanisms.
     */
    public static void setupCADMechanisms(PhysicsWorld world) {{
"""
        
        for mechanism in mechanisms:
            code += f"        // {mechanism.name}\n"
            code += f"        Generated{mechanism.name.replace('-', '_')}.addToWorld(world);\n"
        
        code += """    }
}
"""
        return code
    
    def _generate_java_mechanism_class(
        self,
        mechanism: GroupedMechanism,
        package_name: str
    ) -> str:
        """Generate Java class for a mechanism."""
        class_name = f"Generated{mechanism.name.replace('-', '_')}"
        
        code = f"""package {package_name};

import frc.robot.sim.*;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Auto-generated mechanism: {mechanism.name}
 * Type: {mechanism.mechanism_type.value}
 * Components: {len(mechanism.components)}
 */
public class {class_name} {{
    
    public static final String MECHANISM_NAME = "{mechanism.name}";
    public static final String MECHANISM_TYPE = "{mechanism.mechanism_type.value}";
    
    private static PhysicsBody rootBody;
"""
        
        # Generate component declarations
        code += "\n    // Component bodies\n"
        for component in mechanism.components:
            var_name = component.name.replace("-", "_").lower()
            code += f"    private static PhysicsBody {var_name};\n"
        
        code += f"""
    
    /**
     * Add this mechanism to the physics world.
     */
    public static void addToWorld(PhysicsWorld world) {{
        rootBody = world.addBody(MECHANISM_NAME, {mechanism.components[0].mass});
        
"""
        
        # Generate component setup
        for component in mechanism.components[1:]:
            var_name = component.name.replace("-", "_").lower()
            code += f"        {var_name} = world.addBody(\"{component.name}\", {component.mass});\n"
        
        code += """    }
}
"""
        return code
    
    def export_to_python_dict(
        self,
        mechanisms: List[GroupedMechanism]
    ) -> Dict[str, Any]:
        """Export mechanisms as Python dictionary.
        
        Useful for programmatic access to CAD data.
        
        Args:
            mechanisms: List of mechanisms to export
        
        Returns:
            Dictionary with mechanism data
        """
        export_dict = {
            "mechanisms": []
        }
        
        for mechanism in mechanisms:
            mech_dict = self._mechanism_to_json(mechanism, include_geometry=False)
            export_dict["mechanisms"].append(mech_dict)
        
        return export_dict
    
    def export_summary_report(
        self,
        mechanisms: List[GroupedMechanism],
        output_path: str
    ) -> bool:
        """Export human-readable summary report.
        
        Args:
            mechanisms: List of mechanisms to summarize
            output_path: Path to output report file
        
        Returns:
            True if export successful
        """
        output_path = Path(output_path)
        
        try:
            report = "# CAD Import Summary Report\\n\\n"
            report += f"**Total Mechanisms:** {len(mechanisms)}\\n\\n"
            
            # Statistics
            total_components = sum(len(m.components) for m in mechanisms)
            total_joints = sum(len(m.joints) for m in mechanisms)
            total_mass = sum(
                sum(c.mass for c in m.components) for m in mechanisms
            )
            
            report += "## Statistics\\n"
            report += f"- Total Components: {total_components}\\n"
            report += f"- Total Joints: {total_joints}\\n"
            report += f"- Total Mass: {total_mass:.2f} kg\\n\\n"
            
            # Mechanism details
            report += "## Mechanisms\\n\\n"
            for mechanism in mechanisms:
                report += f"### {mechanism.name}\\n"
                report += f"- **Type:** {mechanism.mechanism_type.value}\\n"
                report += f"- **Components:** {len(mechanism.components)}\\n"
                report += f"- **Joints:** {len(mechanism.joints)}\\n"
                report += f"- **Total Mass:** {sum(c.mass for c in mechanism.components):.2f} kg\\n\\n"
                
                report += "#### Components\\n"
                for comp in mechanism.components:
                    material = comp.material or DEFAULT_MATERIAL
                    report += f"- `{comp.name}`: {comp.mass:.3f} kg ({material})\\n"
                
                report += "\\n"
            
            output_path.parent.mkdir(parents=True, exist_ok=True)
            output_path.write_text(report)
            logger.info(f"Exported summary report: {output_path}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to export summary report: {e}")
            return False


class UniversalCADExporter:
    """High-level exporter supporting multiple formats."""
    
    def __init__(self, material_system: Optional[MaterialSystem] = None):
        """Initialize universal exporter."""
        self.exporter = CADExporter(material_system)
    
    def export(
        self,
        mechanisms: List[GroupedMechanism],
        output_path: str,
        format: ExportFormat = ExportFormat.JSON,
        **kwargs
    ) -> bool:
        """Export mechanisms in specified format.
        
        Args:
            mechanisms: List of mechanisms to export
            output_path: Output file/directory path
            format: Export format
            **kwargs: Format-specific options
        
        Returns:
            True if export successful
        """
        if format == ExportFormat.JSON:
            return self.exporter.export_to_json(mechanisms, output_path, **kwargs)
        elif format == ExportFormat.GLTF:
            logger.error("glTF export not yet implemented")
            return False
        elif format == ExportFormat.URDF:
            logger.error("URDF export not yet implemented")
            return False
        elif format == ExportFormat.CUSTOM:
            # Custom Java codegen format
            return self.exporter.export_to_java_codegen(mechanisms, output_path, **kwargs)
        else:
            logger.error(f"Unknown export format: {format}")
            return False
