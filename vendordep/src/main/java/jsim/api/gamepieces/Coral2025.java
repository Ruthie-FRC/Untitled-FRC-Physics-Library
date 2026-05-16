package jsim.api.gamepieces;

import jsim.api.GamePieceState;
import jsim.api.RectangularPrismGamePiecePhysics;
import edu.wpi.first.math.geometry.Pose3d;

/**
 * Implementation of 2025 Game Piece layout bounds.
 */
public class Coral2025 extends GamePieceState {

    /**
     * Creates a new Coral2025 game piece.
     *
     * <p>The coral is approximated as a rectangular-prism hitbox matching the
     * PVC pipe envelope: 11.875 in long, 4.5 in outer diameter, and about
     * 1.5-1.7 lb mass.
     */
    public Coral2025() {
        super(new RectangularPrismGamePiecePhysics(0.725, 0.301625, 0.1143, 0.1143));
    }

    /**
     * Executes placing the piece onto a branch node.
     * @param branchTarget The 3D position target for placing the coral.
     */
    public void place(Pose3d branchTarget) {
        this.position = branchTarget;
        this.lifecycle = Lifecycle.INTERACTING; // Before resolving definitively securely
    }
}
