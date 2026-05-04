"""
UI Overlays for the viewer plugin.
Handles drawing trajectory splines, collision boxes, and text HUDs.
"""

class OverlayManager:
    def __init__(self):
        self.overlays = []

    def add_overlay(self, overlay_type: str, data: dict):
        """Adds an overlay element to be rendered next frame."""
        self.overlays.append({"type": overlay_type, "data": data})

    def draw_overlays(self):
        """Dispatches drawing commands for all queued overlays."""
        # Clean up queue after drawing
        self.overlays.clear()
