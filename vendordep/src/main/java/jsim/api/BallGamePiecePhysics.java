package jsim.api;

/**
 * Mass and hitbox metadata for a spherical or ball-shaped game piece.
 */
public class BallGamePiecePhysics extends GamePiecePhysics {
    /** Ball mass in kilograms. */
    public final double massKg;
    /** Ball radius in meters. */
    public final double radiusMeters;

    /**
     * Creates a spherical game piece wrapper from radius.
     *
     * @param massKg mass in kilograms
     * @param radiusMeters radius in meters
     */
    public BallGamePiecePhysics(double massKg, double radiusMeters) {
        this.massKg = massKg;
        this.radiusMeters = radiusMeters;
    }

    /**
     * Creates a spherical game piece wrapper from diameter.
     *
     * @param massKg mass in kilograms
     * @param diameterMeters diameter in meters
     * @return spherical game piece wrapper
     */
    public static BallGamePiecePhysics fromDiameter(double massKg, double diameterMeters) {
        return new BallGamePiecePhysics(massKg, diameterMeters * 0.5);
    }

    /**
     * Gets the ball diameter in meters.
     *
     * @return diameter in meters
     */
    public double getDiameterMeters() {
        return radiusMeters * 2.0;
    }
}