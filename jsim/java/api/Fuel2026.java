package api;

/**
 * Represents a 2026 Fuel game piece.
 */
public class Fuel2026 extends GamePieceState {
    /**
     * Constructs a Fuel2026 game piece.
     */
    public Fuel2026() { super(GamePieceType.FUEL); }

    /**
     * Simulates shooting the fuel game piece.
     * @param relativeStart The relative start position.
     * @param timeOfFlight The time of flight.
     * @param exitAngle The exit angle.
     */
    public void shoot(Translation3d relativeStart, double timeOfFlight, Rotation3d exitAngle) {
        setExitAngle(exitAngle);
        // TODO: implement domain-specific prediction if required
    }
}
