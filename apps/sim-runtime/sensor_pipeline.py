"""
Simulated sensor pipeline for JSim runtime. 
Calculates and publishes sensor data such as encoders, gyros, and vision targets over NT4.
"""

import logging

class SensorPipeline:
    def __init__(self):
        self.sensors = []

    def register_sensor(self, sensor):
        """Registers a sensor for runtime execution."""
        self.sensors.append(sensor)
        logging.info(f"Registered sensor: {sensor}")

    def update(self, dt: float):
        """Updates all registered sensors."""
        for sensor in self.sensors:
            # Placeholder for sensor computation
            pass

    def publish(self):
        """Publishes the updated sensor telemetry via NT4."""
        pass
