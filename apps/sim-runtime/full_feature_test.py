from __future__ import annotations

from pathlib import Path

from main_loop import ROOT, main, run_simulation
from robot_loader import load_default_scenario
from sim_options import GraphicsQuality, SimulationFeatures, SimulationRunOptions


def assert_true(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def run() -> None:
    scenario = load_default_scenario()

    low = SimulationRunOptions(
        dt_s=0.004,
        duration_s=0.08,
        quality=GraphicsQuality.LOW,
        features=SimulationFeatures(show_velocity_vectors=False, show_paths=False),
    )
    high = SimulationRunOptions(
        dt_s=0.004,
        duration_s=0.08,
        quality=GraphicsQuality.HIGH,
        features=SimulationFeatures(),
    )

    timeline_low, packets_low = run_simulation(scenario, run_options=low)
    timeline_high, packets_high = run_simulation(scenario, run_options=high)

    assert_true(len(timeline_low) > 0, "low-quality simulation must produce frames")
    assert_true(len(timeline_high) == len(timeline_low), "quality should not change frame count")
    assert_true(len(packets_low) == len(timeline_low), "packet/frame count mismatch")

    main()

    out_dir = ROOT / "build" / "sim-visual"
    dashboard = out_dir / "dashboard.html"
    bindings = out_dir / "control_bindings.html"
    png = out_dir / "arena_frame.png"
    packets = out_dir / "sensor_packets.jsonl"
    timeline_json = out_dir / "timeline_frames.json"

    for path in [dashboard, bindings, png, packets, timeline_json]:
        assert_true(path.exists(), f"expected output not generated: {path}")

    dashboard_text = dashboard.read_text(encoding="utf-8")
    assert_true("Simulation Settings" in dashboard_text, "dashboard missing settings sidebar")
    assert_true("Open Driver/Co-Driver Bindings" in dashboard_text, "dashboard missing bindings link")

    bindings_text = bindings.read_text(encoding="utf-8")
    assert_true("Driver and Co-Driver Control Bindings" in bindings_text, "bindings page missing title")
    assert_true("Save Bindings" in bindings_text, "bindings page missing save control")

    print("Full feature test passed.")


if __name__ == "__main__":
    run()
