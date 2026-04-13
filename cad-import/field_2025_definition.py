"""2025 season field definition (REEFSCAPE, welded field)."""

from typing import Dict, Any


class Field2025WeldedDefinition:
    """REEFSCAPE 2025 welded field definition.

    2025 FRC Game: REEFSCAPE
    Field dimensions: 17.548m x 8.052m

    Key elements:
    - Coral reef structures
    - Algae removal zones
    - Barge zones
    """

    FIELD_LENGTH = 17.548
    FIELD_WIDTH = 8.052

    @staticmethod
    def get_field_definition() -> Dict[str, Any]:
        """Get complete field definition."""
        return {
            "year": 2025,
            "game": "REEFSCAPE",
            "field_variant": "welded",
            "field_dimensions": {
                "length": Field2025WeldedDefinition.FIELD_LENGTH,
                "width": Field2025WeldedDefinition.FIELD_WIDTH,
            },
            "tags": [
                {
                    "ID": 1,
                    "pose": {
                        "translation": {"x": 16.697198, "y": 0.65532, "z": 1.4859},
                        "rotation": {"quaternion": {"W": 0.4539904997395468, "X": 0.0, "Y": 0.0, "Z": 0.8910065241883678}},
                    },
                },
                {
                    "ID": 2,
                    "pose": {
                        "translation": {"x": 16.697198, "y": 7.3964799999999995, "z": 1.4859},
                        "rotation": {"quaternion": {"W": -0.45399049973954675, "X": -0.0, "Y": 0.0, "Z": 0.8910065241883679}},
                    },
                },
                {
                    "ID": 3,
                    "pose": {
                        "translation": {"x": 11.560809999999998, "y": 8.05561, "z": 1.30175},
                        "rotation": {"quaternion": {"W": -0.7071067811865475, "X": -0.0, "Y": 0.0, "Z": 0.7071067811865476}},
                    },
                },
                {
                    "ID": 4,
                    "pose": {
                        "translation": {"x": 9.276079999999999, "y": 6.137656, "z": 1.8679160000000001},
                        "rotation": {"quaternion": {"W": 0.9659258262890683, "X": 0.0, "Y": 0.25881904510252074, "Z": 0.0}},
                    },
                },
                {
                    "ID": 5,
                    "pose": {
                        "translation": {"x": 9.276079999999999, "y": 1.914906, "z": 1.8679160000000001},
                        "rotation": {"quaternion": {"W": 0.9659258262890683, "X": 0.0, "Y": 0.25881904510252074, "Z": 0.0}},
                    },
                },
                {
                    "ID": 6,
                    "pose": {
                        "translation": {"x": 13.474446, "y": 3.3063179999999996, "z": 0.308102},
                        "rotation": {"quaternion": {"W": -0.8660254037844387, "X": -0.0, "Y": 0.0, "Z": 0.49999999999999994}},
                    },
                },
                {
                    "ID": 7,
                    "pose": {
                        "translation": {"x": 13.890498, "y": 4.0259, "z": 0.308102},
                        "rotation": {"quaternion": {"W": 1.0, "X": 0.0, "Y": 0.0, "Z": 0.0}},
                    },
                },
                {
                    "ID": 8,
                    "pose": {
                        "translation": {"x": 13.474446, "y": 4.745482, "z": 0.308102},
                        "rotation": {"quaternion": {"W": 0.8660254037844387, "X": 0.0, "Y": 0.0, "Z": 0.49999999999999994}},
                    },
                },
                {
                    "ID": 9,
                    "pose": {
                        "translation": {"x": 12.643358, "y": 4.745482, "z": 0.308102},
                        "rotation": {"quaternion": {"W": 0.5000000000000001, "X": 0.0, "Y": 0.0, "Z": 0.8660254037844386}},
                    },
                },
                {
                    "ID": 10,
                    "pose": {
                        "translation": {"x": 12.227305999999999, "y": 4.0259, "z": 0.308102},
                        "rotation": {"quaternion": {"W": 6.123233995736766e-17, "X": 0.0, "Y": 0.0, "Z": 1.0}},
                    },
                },
                {
                    "ID": 11,
                    "pose": {
                        "translation": {"x": 12.643358, "y": 3.3063179999999996, "z": 0.308102},
                        "rotation": {"quaternion": {"W": -0.4999999999999998, "X": -0.0, "Y": 0.0, "Z": 0.8660254037844387}},
                    },
                },
                {
                    "ID": 12,
                    "pose": {
                        "translation": {"x": 0.851154, "y": 0.65532, "z": 1.4859},
                        "rotation": {"quaternion": {"W": 0.8910065241883679, "X": 0.0, "Y": 0.0, "Z": 0.45399049973954675}},
                    },
                },
                {
                    "ID": 13,
                    "pose": {
                        "translation": {"x": 0.851154, "y": 7.3964799999999995, "z": 1.4859},
                        "rotation": {"quaternion": {"W": -0.8910065241883678, "X": -0.0, "Y": 0.0, "Z": 0.45399049973954686}},
                    },
                },
                {
                    "ID": 14,
                    "pose": {
                        "translation": {"x": 8.272272, "y": 6.137656, "z": 1.8679160000000001},
                        "rotation": {"quaternion": {"W": 5.914589856893349e-17, "X": -0.25881904510252074, "Y": 1.5848095757158825e-17, "Z": 0.9659258262890683}},
                    },
                },
                {
                    "ID": 15,
                    "pose": {
                        "translation": {"x": 8.272272, "y": 1.914906, "z": 1.8679160000000001},
                        "rotation": {"quaternion": {"W": 5.914589856893349e-17, "X": -0.25881904510252074, "Y": 1.5848095757158825e-17, "Z": 0.9659258262890683}},
                    },
                },
                {
                    "ID": 16,
                    "pose": {
                        "translation": {"x": 5.9875419999999995, "y": -0.0038099999999999996, "z": 1.30175},
                        "rotation": {"quaternion": {"W": 0.7071067811865476, "X": 0.0, "Y": 0.0, "Z": 0.7071067811865476}},
                    },
                },
                {
                    "ID": 17,
                    "pose": {
                        "translation": {"x": 4.073905999999999, "y": 3.3063179999999996, "z": 0.308102},
                        "rotation": {"quaternion": {"W": -0.4999999999999998, "X": -0.0, "Y": 0.0, "Z": 0.8660254037844387}},
                    },
                },
                {
                    "ID": 18,
                    "pose": {
                        "translation": {"x": 3.6576, "y": 4.0259, "z": 0.308102},
                        "rotation": {"quaternion": {"W": 6.123233995736766e-17, "X": 0.0, "Y": 0.0, "Z": 1.0}},
                    },
                },
                {
                    "ID": 19,
                    "pose": {
                        "translation": {"x": 4.073905999999999, "y": 4.745482, "z": 0.308102},
                        "rotation": {"quaternion": {"W": 0.5000000000000001, "X": 0.0, "Y": 0.0, "Z": 0.8660254037844386}},
                    },
                },
                {
                    "ID": 20,
                    "pose": {
                        "translation": {"x": 4.904739999999999, "y": 4.745482, "z": 0.308102},
                        "rotation": {"quaternion": {"W": 0.8660254037844387, "X": 0.0, "Y": 0.0, "Z": 0.49999999999999994}},
                    },
                },
                {
                    "ID": 21,
                    "pose": {
                        "translation": {"x": 5.321046, "y": 4.0259, "z": 0.308102},
                        "rotation": {"quaternion": {"W": 1.0, "X": 0.0, "Y": 0.0, "Z": 0.0}},
                    },
                },
                {
                    "ID": 22,
                    "pose": {
                        "translation": {"x": 4.904739999999999, "y": 3.3063179999999996, "z": 0.308102},
                        "rotation": {"quaternion": {"W": -0.8660254037844387, "X": -0.0, "Y": 0.0, "Z": 0.49999999999999994}},
                    },
                },
            ],
            "elements": [
                # Reefs (left)
                {
                    "name": "reef_left",
                    "type": "reef",
                    "pose": {
                        "x": 2.5,
                        "y": 4.105,
                        "z": 0.0,
                        "roll": 0.0,
                        "pitch": 0.0,
                        "yaw": 0.0,
                    },
                    "dimensions": {
                        "length": 2.0,
                        "width": 2.0,
                    },
                    "material": "plastic",
                    "mass": 50.0,
                },
                # Reefs (right)
                {
                    "name": "reef_right",
                    "type": "reef",
                    "pose": {
                        "x": 14.04,
                        "y": 4.105,
                        "z": 0.0,
                        "roll": 0.0,
                        "pitch": 0.0,
                        "yaw": 0.0,
                    },
                    "dimensions": {
                        "length": 2.0,
                        "width": 2.0,
                    },
                    "material": "plastic",
                    "mass": 50.0,
                },
                # Barge (blue)
                {
                    "name": "barge_blue",
                    "type": "platform",
                    "pose": {
                        "x": 1.0,
                        "y": 0.5,
                        "z": 0.0,
                        "roll": 0.0,
                        "pitch": 0.0,
                        "yaw": 0.0,
                    },
                    "dimensions": {
                        "length": 3.66,
                        "width": 2.74,
                    },
                    "material": "aluminum",
                    "mass": 100.0,
                    "alliance": "blue",
                },
                # Barge (red)
                {
                    "name": "barge_red",
                    "type": "platform",
                    "pose": {
                        "x": 15.54,
                        "y": 7.71,
                        "z": 0.0,
                        "roll": 0.0,
                        "pitch": 0.0,
                        "yaw": 3.14159,
                    },
                    "dimensions": {
                        "length": 3.66,
                        "width": 2.74,
                    },
                    "material": "aluminum",
                    "mass": 100.0,
                    "alliance": "red",
                },
            ],
            "game_pieces": [
                {
                    "type": "coral",
                    "initial_count": 12,
                    "mass": 0.567,  # kg
                    "material": "plastic",
                    "diameter": 0.178,
                }
            ],
            "constraints": {
                "min_robot_size": 0.15,
                "max_robot_size": 1.20,
                "max_robot_height": 1.90,
                "max_robot_mass": 70.0,
            }
        }
