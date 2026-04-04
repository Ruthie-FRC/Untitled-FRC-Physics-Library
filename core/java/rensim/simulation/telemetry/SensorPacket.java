package rensim.simulation.telemetry;

import java.util.List;

/**
 * Frame-level telemetry packet exported by runtime simulation integrations.
 *
 * @param tick simulation tick index
 * @param timeSeconds simulation time in seconds
 * @param contactCount number of contacts resolved on this tick
 * @param bodies per-body telemetry snapshots
 */
public record SensorPacket(
    int tick,
    double timeSeconds,
    int contactCount,
    List<BodyTelemetry> bodies) {
  public SensorPacket {
    bodies = List.copyOf(bodies);
  }
}
