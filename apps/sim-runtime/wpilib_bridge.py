"""JSim runtime bridge helpers for WPILib-style integrations.

This module owns the runtime-side pose tracking and snapshot plumbing so CAD
import generation does not need to emit robot-code integration shims.
"""

from dataclasses import dataclass
from threading import RLock
from typing import List


@dataclass
class Pose3dState:
	"""Thread-safe pose payload used by the runtime bridge."""

	x: float = 0.0
	y: float = 0.0
	z: float = 0.0
	roll: float = 0.0
	pitch: float = 0.0
	yaw: float = 0.0

	def to_list(self) -> List[float]:
		return [self.x, self.y, self.z, self.roll, self.pitch, self.yaw]


class JSimPose3dUpdater:
	"""Runtime-owned pose updater for JSim integrations.

	The simulator can feed this bridge directly instead of generating a custom
	updater inside downstream robot code.
	"""

	def __init__(self) -> None:
		self._lock = RLock()
		self._pose = Pose3dState()

	def update_pose(self, pose: Pose3dState) -> None:
		with self._lock:
			self._pose = pose

	def update_pose_array(self, pose_array: List[float]) -> bool:
		if len(pose_array) != 6:
			return False

		with self._lock:
			self._pose = Pose3dState(*pose_array)
		return True

	def get_pose(self) -> Pose3dState:
		with self._lock:
			return self._pose

	def get_pose_array(self) -> List[float]:
		return self.get_pose().to_list()


class WPILibBridge:
	"""Minimal runtime bridge for JSim-owned state exchange."""

	def __init__(self) -> None:
		self.robot_pose_updater = JSimPose3dUpdater()

	def publish_robot_pose(self, pose: Pose3dState) -> None:
		self.robot_pose_updater.update_pose(pose)

	def read_robot_pose(self) -> Pose3dState:
		return self.robot_pose_updater.get_pose()
