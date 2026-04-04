from __future__ import annotations

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[2]
VIEWER_PLUGIN_DIR = ROOT / "apps" / "viewer-plugin"
if str(VIEWER_PLUGIN_DIR) not in sys.path:
	sys.path.insert(0, str(VIEWER_PLUGIN_DIR))

from renderer import render_timeline_matplotlib  # noqa: E402
from timeline import BodyFrame, SimFrame, Timeline  # noqa: E402


__all__ = [
	"BodyFrame",
	"SimFrame",
	"Timeline",
	"render_timeline_matplotlib",
]
