"""
Grouped mechanism extraction and management.

Handles extraction of grouped mechanisms from CAD assemblies.
From discussion #36: Only simulate groups/collections, preferably only ones that move in OnShape sim.
"""

from dataclasses import dataclass, field
from typing import List, Dict, Optional, Set, Any
from enum import Enum
import logging

logger = logging.getLogger(__name__)


class MechanismType(Enum):
    """Types of mechanisms that can be simulated."""
    MOTOR_DRIVEN = "motor_driven"
    LINKED_ASSEMBLY = "linked_assembly"  # 4-bar, linkages
    ROTATING_JOINT = "rotating_joint"
    LINEAR_JOINT = "linear_joint"
    FIXED = "fixed"
    UNKNOWN = "unknown"


@dataclass
class Component:
    """Represents a component/part in a mechanism."""
    name: str
    part_number: Optional[str] = None
    material: Optional[str] = None
    mass: float = 0.0
    volume: float = 0.0
    collision_enabled: bool = True
    is_fastener: bool = False
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass
class Joint:
    """Represents a joint between components."""
    name: str
    joint_type: str  # "revolute", "prismatic", "fixed", etc.
    parent_component: str
    child_component: str
    axis: tuple = (0, 0, 1)  # Default: Z axis
    limits: Optional[tuple] = None  # (min, max) in degrees or meters
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass
class GroupedMechanism:
    """Represents a grouped mechanism ready for export."""
    name: str
    mechanism_type: MechanismType
    components: List[Component]
    joints: List[Joint]
    root_component: str
    metadata: Dict[str, Any] = field(default_factory=dict)
    
    def get_component(self, name: str) -> Optional[Component]:
        """Get component by name."""
        for comp in self.components:
            if comp.name == name:
                return comp
        return None
    
    def get_static_mass(self) -> float:
        """Get total mass of mechanism."""
        return sum(c.mass for c in self.components)
    
    def validate(self) -> tuple[bool, List[str]]:
        """Validate mechanism integrity.
        
        Returns:
            (is_valid, list of warnings/errors)
        """
        issues = []
        
        if not self.components:
            issues.append(f"Mechanism '{self.name}' has no components")
        
        if not self.root_component:
            issues.append(f"Mechanism '{self.name}' has no root component")
        
        component_names = {c.name for c in self.components}
        if self.root_component not in component_names:
            issues.append(f"Root component '{self.root_component}' not in components list")
        
        # Validate joint references
        for joint in self.joints:
            if joint.parent_component not in component_names:
                issues.append(f"Joint '{joint.name}' references unknown parent '{joint.parent_component}'")
            if joint.child_component not in component_names:
                issues.append(f"Joint '{joint.name}' references unknown child '{joint.child_component}'")
        
        # Check for zero mass components
        zero_mass = [c.name for c in self.components if c.mass == 0.0]
        if zero_mass:
            issues.append(f"Zero mass components: {zero_mass} (will be treated as fixed)")
        
        is_valid = len(issues) == 0
        return is_valid, issues


class MechanismExtractor:
    """Extracts grouped mechanisms from CAD assemblies."""
    
    def __init__(self):
        """Initialize mechanism extractor."""
        self.mechanisms: List[GroupedMechanism] = []
        self.ignored_components: Set[str] = set()
    
    def extract_mechanism_from_group(
        self,
        group_name: str,
        components: List[Dict[str, Any]],
        joints: Optional[List[Dict[str, Any]]] = None
    ) -> Optional[GroupedMechanism]:
        """Extract a mechanism from a component group.
        
        Args:
            group_name: Name of the group/mechanism
            components: List of component definitions
            joints: List of joint definitions
        
        Returns:
            GroupedMechanism or None if invalid
        """
        if not components:
            logger.warning(f"Mechanism '{group_name}' has no components, skipping")
            return None
        
        extracted_components = []
        for comp_data in components:
            component = Component(
                name=comp_data.get("name", "unknown"),
                part_number=comp_data.get("part_number"),
                material=comp_data.get("material"),
                mass=comp_data.get("mass", 0.0),
                volume=comp_data.get("volume", 0.0),
                collision_enabled=comp_data.get("collision_enabled", True),
                is_fastener=comp_data.get("is_fastener", False),
                metadata=comp_data.get("metadata", {})
            )
            extracted_components.append(component)
        
        extracted_joints = []
        if joints:
            for joint_data in joints:
                joint = Joint(
                    name=joint_data.get("name", "unknown"),
                    joint_type=joint_data.get("type", "fixed"),
                    parent_component=joint_data.get("parent"),
                    child_component=joint_data.get("child"),
                    axis=joint_data.get("axis", (0, 0, 1)),
                    limits=joint_data.get("limits"),
                    metadata=joint_data.get("metadata", {})
                )
                extracted_joints.append(joint)
        
        # Determine mechanism type
        mech_type = self._infer_mechanism_type(extracted_components, extracted_joints)
        
        # Use first component as root
        root = extracted_components[0].name
        
        mechanism = GroupedMechanism(
            name=group_name,
            mechanism_type=mech_type,
            components=extracted_components,
            joints=extracted_joints,
            root_component=root
        )
        
        is_valid, issues = mechanism.validate()
        for issue in issues:
            logger.warning(f"Mechanism validation: {issue}")
        
        if is_valid:
            self.mechanisms.append(mechanism)
            logger.info(f"Extracted mechanism: {group_name} ({mech_type.value}, {len(extracted_components)} components)")
        
        return mechanism if is_valid else None
    
    def _infer_mechanism_type(
        self,
        components: List[Component],
        joints: List[Joint]
    ) -> MechanismType:
        """Infer mechanism type from structure.
        
        Args:
            components: List of components
            joints: List of joints
        
        Returns:
            Inferred mechanism type
        """
        if not joints:
            return MechanismType.FIXED
        
        joint_types = {j.joint_type for j in joints}
        
        if "revolute" in joint_types:
            return MechanismType.ROTATING_JOINT
        elif "prismatic" in joint_types:
            return MechanismType.LINEAR_JOINT
        elif len(joints) > 1:
            return MechanismType.LINKED_ASSEMBLY
        
        return MechanismType.UNKNOWN
    
    def mark_component_ignored(self, component_name: str):
        """Mark a component to be ignored in exports.
        
        Args:
            component_name: Name of component to ignore
        """
        self.ignored_components.add(component_name)
        logger.debug(f"Marked component '{component_name}' as ignored")
    
    def get_mechanisms_to_simulate(self, accuracy_level) -> List[GroupedMechanism]:
        """Get list of mechanisms that should be simulated at given accuracy level.
        
        Args:
            accuracy_level: AccuracyLevel to filter by
        
        Returns:
            List of mechanisms suitable for simulation
        """
        mechanisms_to_simulate = []
        
        from .config import ACCURACY_CONFIGS
        config = ACCURACY_CONFIGS[accuracy_level]
        
        for mechanism in self.mechanisms:
            # Filter by fasteners if needed
            if not config.simulate_fasteners:
                has_only_fasteners = all(c.is_fastener for c in mechanism.components)
                if has_only_fasteners:
                    logger.debug(f"Skipping fastener-only mechanism: {mechanism.name}")
                    continue
            
            # Filter by mass threshold
            has_meaningful_mass = any(c.mass >= config.min_mass_threshold for c in mechanism.components)
            if not has_meaningful_mass:
                logger.debug(f"Skipping low-mass mechanism: {mechanism.name}")
                continue
            
            mechanisms_to_simulate.append(mechanism)
        
        return mechanisms_to_simulate
    
    def get_summary(self) -> Dict[str, Any]:
        """Get summary of extracted mechanisms."""
        return {
            "total_mechanisms": len(self.mechanisms),
            "mechanism_types": {
                mt.value: len([m for m in self.mechanisms if m.mechanism_type == mt])
                for mt in MechanismType
            },
            "total_components": sum(len(m.components) for m in self.mechanisms),
            "total_joints": sum(len(m.joints) for m in self.mechanisms),
            "ignored_components": len(self.ignored_components)
        }
