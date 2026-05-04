"""
Playback timeline scrubber for the viewer plugin.
Handles simulation recording, pausing, and playing back states.
"""

class Timeline:
    def __init__(self):
        self.history = []
        self.current_frame = 0
        self.is_playing = True

    def record_state(self, state: dict):
        """Records a physics state frame into history."""
        self.history.append(state)
        if self.is_playing:
            self.current_frame = len(self.history) - 1

    def set_frame(self, frame_idx: int):
        """Scrubs to a specific point in time."""
        if 0 <= frame_idx < len(self.history):
            self.current_frame = frame_idx
            self.is_playing = False

    def get_current_state(self):
        """Returns the state relative to the scrubber playhead."""
        if not self.history:
            return None
        return self.history[self.current_frame]
