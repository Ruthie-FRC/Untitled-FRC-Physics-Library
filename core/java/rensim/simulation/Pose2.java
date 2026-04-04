package rensim.simulation;

/**
 * Lightweight 2D pose for field-level simulations.
 *
 * @param xMeters x position in meters
 * @param yMeters y position in meters
 * @param yawRad heading in radians
 */
public record Pose2(double xMeters, double yMeters, double yawRad) {}