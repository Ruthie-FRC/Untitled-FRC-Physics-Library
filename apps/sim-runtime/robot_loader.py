"""
Robot asset loader for JSim runtime.
Handles reading robot configuration and instantiating physics bodies.
"""

import logging

class RobotLoader:
    def __init__(self, asset_path: str):
        self.asset_path = asset_path
        self.bodies = []

    def load(self):
        """Loads the robot model from the specified asset path."""
        logging.info(f"Loading robot from {self.asset_path}")
        # TODO: Integrate with CAD import systems (URDF/STL loaders)
        return True

    def get_bodies(self):
        """Returns the loaded rigid bodies for the simulator to take ownership of."""
        return self.bodies
