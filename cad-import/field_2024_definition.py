"""2024 season field definition (CRESCENDO)."""

from typing import Dict, Any


class Field2024Definition:
    """CRESCENDO 2024 field definition.

    2024 FRC Game: CRESCENDO
    Field dimensions: 16.541m x 8.211m

    Key elements:
    - Stage (centerfield)
    - Speaker amp zone (wings)
    - Note drop zones
    - Wings for robot positioning
    """

    FIELD_LENGTH = 16.541  # meters
    FIELD_WIDTH = 8.211    # meters

    @staticmethod
    def get_field_definition() -> Dict[str, Any]:
        """Get complete field definition as JSON-serializable dict."""
        return {
            "year": 2024,
            "game": "CRESCENDO",
            "field_dimensions": {
                "length": Field2024Definition.FIELD_LENGTH,
                "width": Field2024Definition.FIELD_WIDTH,
            },
            "tags": [
                {
                    "ID": 1,
                    "pose": {
                        "translation": {"x": 15.079471999999997, "y": 0.24587199999999998, "z": 1.355852},
                        "rotation": {"quaternion": {"W": 0.5000000000000001, "X": 0.0, "Y": 0.0, "Z": 0.8660254037844386}},
                    },
                },
                {
                    "ID": 2,
                    "pose": {
                        "translation": {"x": 16.185134, "y": 0.883666, "z": 1.355852},
                        "rotation": {"quaternion": {"W": 0.5000000000000001, "X": 0.0, "Y": 0.0, "Z": 0.8660254037844386}},
                    },
                },
                {
                    "ID": 3,
                    "pose": {
                        "translation": {"x": 16.579342, "y": 4.982717999999999, "z": 1.4511020000000001},
                        "rotation": {"quaternion": {"W": 6.123233995736766e-17, "X": 0.0, "Y": 0.0, "Z": 1.0}},
                    },
                },
                {
                    "ID": 4,
                    "pose": {
                        "translation": {"x": 16.579342, "y": 5.547867999999999, "z": 1.4511020000000001},
                        "rotation": {"quaternion": {"W": 6.123233995736766e-17, "X": 0.0, "Y": 0.0, "Z": 1.0}},
                    },
                },
                {
                    "ID": 5,
                    "pose": {
                        "translation": {"x": 14.700757999999999, "y": 8.2042, "z": 1.355852},
                        "rotation": {"quaternion": {"W": -0.7071067811865475, "X": -0.0, "Y": 0.0, "Z": 0.7071067811865476}},
                    },
                },
                {
                    "ID": 6,
                    "pose": {
                        "translation": {"x": 1.8415, "y": 8.2042, "z": 1.355852},
                        "rotation": {"quaternion": {"W": -0.7071067811865475, "X": -0.0, "Y": 0.0, "Z": 0.7071067811865476}},
                    },
                },
                {
                    "ID": 7,
                    "pose": {
                        "translation": {"x": -0.038099999999999995, "y": 5.547867999999999, "z": 1.4511020000000001},
                        "rotation": {"quaternion": {"W": 1.0, "X": 0.0, "Y": 0.0, "Z": 0.0}},
                    },
                },
                {
                    "ID": 8,
                    "pose": {
                        "translation": {"x": -0.038099999999999995, "y": 4.982717999999999, "z": 1.4511020000000001},
                        "rotation": {"quaternion": {"W": 1.0, "X": 0.0, "Y": 0.0, "Z": 0.0}},
                    },
                },
                {
                    "ID": 9,
                    "pose": {
                        "translation": {"x": 0.356108, "y": 0.883666, "z": 1.355852},
                        "rotation": {"quaternion": {"W": 0.8660254037844387, "X": 0.0, "Y": 0.0, "Z": 0.49999999999999994}},
                    },
                },
                {
                    "ID": 10,
                    "pose": {
                        "translation": {"x": 1.4615159999999998, "y": 0.24587199999999998, "z": 1.355852},
                        "rotation": {"quaternion": {"W": 0.8660254037844387, "X": 0.0, "Y": 0.0, "Z": 0.49999999999999994}},
                    },
                },
                {
                    "ID": 11,
                    "pose": {
                        "translation": {"x": 11.904726, "y": 3.7132259999999997, "z": 1.3208},
                        "rotation": {"quaternion": {"W": -0.8660254037844387, "X": -0.0, "Y": 0.0, "Z": 0.49999999999999994}},
                    },
                },
                {
                    "ID": 12,
                    "pose": {
                        "translation": {"x": 11.904726, "y": 4.49834, "z": 1.3208},
                        "rotation": {"quaternion": {"W": 0.8660254037844387, "X": 0.0, "Y": 0.0, "Z": 0.49999999999999994}},
                    },
                },
                {
                    "ID": 13,
                    "pose": {
                        "translation": {"x": 11.220196, "y": 4.105148, "z": 1.3208},
                        "rotation": {"quaternion": {"W": 6.123233995736766e-17, "X": 0.0, "Y": 0.0, "Z": 1.0}},
                    },
                },
                {
                    "ID": 14,
                    "pose": {
                        "translation": {"x": 5.320792, "y": 4.105148, "z": 1.3208},
                        "rotation": {"quaternion": {"W": 1.0, "X": 0.0, "Y": 0.0, "Z": 0.0}},
                    },
                },
                {
                    "ID": 15,
                    "pose": {
                        "translation": {"x": 4.641342, "y": 4.49834, "z": 1.3208},
                        "rotation": {"quaternion": {"W": 0.5000000000000001, "X": 0.0, "Y": 0.0, "Z": 0.8660254037844386}},
                    },
                },
                {
                    "ID": 16,
                    "pose": {
                        "translation": {"x": 4.641342, "y": 3.7132259999999997, "z": 1.3208},
                        "rotation": {"quaternion": {"W": -0.4999999999999998, "X": -0.0, "Y": 0.0, "Z": 0.8660254037844387}},
                    },
                },
            ],
            "elements": [
                # Stage (center)
                {
                    "name": "stage_platform",
                    "type": "platform",
                    "pose": {
                        "x": 8.27,
                        "y": 4.105,
                        "z": 0.108,  # 4.25 inches high
                        "roll": 0.0,
                        "pitch": 0.0,
                        "yaw": 0.0,
                    },
                    "dimensions": {
                        "length": 2.286,  # 7 feet 6 inches
                        "width": 2.286,
                        "height": 0.108,
                    },
                    "material": "wood",
                    "mass": 50.0,
                },
                # Blue Speaker
                {
                    "name": "speaker_blue",
                    "type": "goal",
                    "pose": {
                        "x": 0.0,
                        "y": 4.105,
                        "z": 2.0,
                        "roll": 0.0,
                        "pitch": 0.0,
                        "yaw": 0.0,
                    },
                    "dimensions": {
                        "opening_width": 1.05,
                        "opening_height": 0.87,
                    },
                    "material": "aluminum",
                    "mass": 100.0,
                    "alliance": "blue",
                },
                # Red Speaker
                {
                    "name": "speaker_red",
                    "type": "goal",
                    "pose": {
                        "x": 16.54,
                        "y": 4.105,
                        "z": 2.0,
                        "roll": 0.0,
                        "pitch": 0.0,
                        "yaw": 3.14159,
                    },
                    "dimensions": {
                        "opening_width": 1.05,
                        "opening_height": 0.87,
                    },
                    "material": "aluminum",
                    "mass": 100.0,
                    "alliance": "red",
                },
                # Amp zones
                {
                    "name": "amp_zone_blue",
                    "type": "zone",
                    "pose": {
                        "x": 1.9,
                        "y": 7.7,
                        "z": 0.0,
                        "roll": 0.0,
                        "pitch": 0.0,
                        "yaw": 0.0,
                    },
                    "dimensions": {
                        "length": 0.8,
                        "width": 0.8,
                    },
                    "material": "painted_metal",
                    "mass": 0.0,  # Not physical
                },
                {
                    "name": "amp_zone_red",
                    "type": "zone",
                    "pose": {
                        "x": 14.64,
                        "y": 7.7,
                        "z": 0.0,
                        "roll": 0.0,
                        "pitch": 0.0,
                        "yaw": 0.0,
                    },
                    "dimensions": {
                        "length": 0.8,
                        "width": 0.8,
                    },
                    "material": "painted_metal",
                    "mass": 0.0,
                },
            ],
            "game_pieces": [
                {
                    "type": "note",
                    "initial_count": 5,  # Notes available on field
                    "mass": 0.235,  # kg
                    "material": "rubber_composite",
                    "diameter": 0.375,  # meters
                }
            ],
            "constraints": {
                "min_robot_size": 0.15,
                "max_robot_size": 1.20,
                "max_robot_height": 1.90,
                "max_robot_mass": 70.0,  # kg
            }
        }
