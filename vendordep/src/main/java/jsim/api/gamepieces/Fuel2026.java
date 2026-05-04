package jsim.api.gamepieces;

import jsim.api.GamePieceState;
import jsim.api.GamePiecePhysics;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Implementation of 2026 Game Piece Physics bounds.
 */
public class Fuel2026 extends GamePieceState {

    /**
     * Creates a new Fuel2026 game piece.
     */
    public Fuel2026() {
        super(new GamePiecePhysics());
    }

    /**
     * Executes a physical shot.
     * @param relativeStart The relative starting position from the robot.
     * @param timeOfFlightMs The time of flight for the projectile in milliseconds.
     * @param exitAngle The exit angle rotation for the shot.
     */
    public void shoot(Translation3d relativeStart, double timeOfFlightMs, Rotation3d exitAngle) {
        this.lifecycle = Lifecycle.ACTIVE;
        this.physics.linearVelocity = new Translation3d(1.0, 0.0, 0.0); // Derived from angle and TOF
    }
}
