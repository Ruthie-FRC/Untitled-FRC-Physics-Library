package jsim.api;

import edu.wpi.first.math.geometry.Translation3d;

/**
 * Mass and hitbox metadata for a rectangular-prism game piece.
 */
public class RectangularPrismGamePiecePhysics extends GamePiecePhysics {
    /** Prism mass in kilograms. */
    public final double massKg;
    /** Prism dimensions in meters, ordered as length, width, height. */
    public final Translation3d dimensionsMeters;

    /**
     * Creates a rectangular-prism game piece wrapper.
     *
     * @param massKg mass in kilograms
     * @param lengthMeters length in meters along the local x axis
     * @param widthMeters width in meters along the local y axis
     * @param heightMeters height in meters along the local z axis
     */
    public RectangularPrismGamePiecePhysics(
            double massKg, double lengthMeters, double widthMeters, double heightMeters) {
        this.massKg = massKg;
        this.dimensionsMeters = new Translation3d(lengthMeters, widthMeters, heightMeters);
    }
}