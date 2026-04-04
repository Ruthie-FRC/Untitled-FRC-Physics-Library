package rensim.simulation.cad;

import java.util.List;
import rensim.Vec3;

/**
 * Lightweight CAD model metadata for simulation overlays and collision approximations.
 */
public record RobotCadModel(
    String modelName,
    String sourcePath,
    List<RobotCadLink> links,
    Vec3 boundsMeters) {

  public record RobotCadLink(String name, double massKg, Vec3 centerOfMassMeters) {}
}
