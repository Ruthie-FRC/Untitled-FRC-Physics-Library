package rensim.simulation.cad;

/**
 * CAD import operation result metadata.
 */
public record CadImportResult(boolean success, String message, RobotCadModel model) {}
