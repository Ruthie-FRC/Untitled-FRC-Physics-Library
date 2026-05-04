"""
Main simulation runtime loop for JSim.
Handles the fixed timestep update and NT4 synchronization.
"""

import time
import logging

class SimLoop:
    def __init__(self, tick_rate_hz: float = 50.0):
        self.tick_rate_hz = tick_rate_hz
        self.dt = 1.0 / tick_rate_hz
        self._running = False

    def start(self):
        """Starts the simulation main loop."""
        self._running = True
        logging.info(f"Starting simulation loop at {self.tick_rate_hz} Hz")
        try:
            self._loop()
        except KeyboardInterrupt:
            self.stop()

    def stop(self):
        """Stops the simulation gracefully."""
        self._running = False
        logging.info("Stopping simulation loop.")

    def _loop(self):
        while self._running:
            start_time = time.perf_counter()
            self._tick()
            elapsed = time.perf_counter() - start_time
            sleep_time = self.dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                logging.warning("Simulation tick trailing behind real-time!")

    def _tick(self):
        """Advances the physics state by one timestep."""
        # TODO: integrate with core physics Driver and NT4 NetworkTables
        pass

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    loop = SimLoop()
    loop.start()
