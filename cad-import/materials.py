"""
Material system for CAD components.

Handles material definition, properties retrieval, and intelligent defaults.
From discussion #36: Teams are bad at defining materials, so assume aluminum if not defined.
"""

from typing import Dict, Tuple, Optional, Any
import logging
from .config import MATERIALS, DEFAULT_MATERIAL

logger = logging.getLogger(__name__)


@staticmethod
def normalize_material_name(name: str) -> str:
    """Normalize material name for lookup."""
    return name.lower().strip().replace(" ", "_").replace("-", "_")


class MaterialSystem:
    """Manages material properties and defaults."""
    
    def __init__(self, custom_materials: Optional[Dict[str, tuple]] = None):
        """Initialize material system.
        
        Args:
            custom_materials: Optional custom material definitions
        """
        self.materials = MATERIALS.copy()
        if custom_materials:
            self.materials.update(custom_materials)
    
    def get_material_properties(self, material_name: Optional[str]) -> Tuple[float, float, float]:
        """Get material properties (density, friction, restitution).
        
        Args:
            material_name: Material name. If None or not found, uses default (aluminum).
        
        Returns:
            Tuple of (density in kg/m³, friction coefficient, restitution)
        """
        if not material_name:
            logger.warning(f"Material not specified, using default: {DEFAULT_MATERIAL}")
            material_name = DEFAULT_MATERIAL
        
        normalized_name = normalize_material_name(material_name)
        
        if normalized_name not in self.materials:
            logger.warning(
                f"Material '{material_name}' not found. Available: {list(self.materials.keys())}. "
                f"Using default: {DEFAULT_MATERIAL}"
            )
            normalized_name = normalize_material_name(DEFAULT_MATERIAL)
        
        density, friction, restitution = self.materials[normalized_name]
        logger.debug(f"Material '{material_name}': ρ={density} kg/m³, μ={friction}, e={restitution}")
        return density, friction, restitution
    
    def calculate_mass(self, volume: float, material_name: Optional[str]) -> float:
        """Calculate component mass from volume and material.
        
        Args:
            volume: Component volume in m³
            material_name: Material name
        
        Returns:
            Mass in kilograms
        """
        density, _, _ = self.get_material_properties(material_name)
        mass = volume * density
        return mass
    
    def is_realistic_mass(self, mass: float, expected_range: Tuple[float, float] = None) -> bool:
        """Check if mass is within realistic range.
        
        Args:
            mass: Component mass in kg
            expected_range: Expected range (min, max). If None, uses defaults.
        
        Returns:
            True if mass is realistic
        """
        from .config import ModelValidationRules
        range_to_check = expected_range or ModelValidationRules.REALISTIC_MASS_RANGE
        min_mass, max_mass = range_to_check
        
        if mass < min_mass or mass > max_mass:
            logger.warning(f"Unrealistic mass detected: {mass} kg (expected {min_mass}-{max_mass} kg)")
            return False
        return True
    
    def add_custom_material(self, name: str, density: float, friction: float, restitution: float):
        """Add custom material definition.
        
        Args:
            name: Material name
            density: Density in kg/m³
            friction: Friction coefficient (0-1)
            restitution: Coefficient of restitution (0-1)
        """
        normalized_name = normalize_material_name(name)
        self.materials[normalized_name] = (density, friction, restitution)
        logger.info(f"Added custom material: {name} (ρ={density}, μ={friction}, e={restitution})")
    
    def list_available_materials(self) -> Dict[str, Tuple[float, float, float]]:
        """Get list of all available materials."""
        return self.materials.copy()
