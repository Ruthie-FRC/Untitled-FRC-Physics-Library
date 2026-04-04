package rensim.simulation.telemetry;

import java.util.Objects;

/**
 * Per-body telemetry payload exported by simulation runtime integrations.
 *
 * @param name body identifier
 * @param xMeters x position in meters
 * @param yMeters y position in meters
 * @param vxMps x velocity in m/s
 * @param vyMps y velocity in m/s
 * @param speedMps planar speed in m/s
 */
public record BodyTelemetry(
    String name,
    double xMeters,
    double yMeters,
    double vxMps,
    double vyMps,
    double speedMps) {
  public BodyTelemetry {
    Objects.requireNonNull(name, "name");
  }
}
