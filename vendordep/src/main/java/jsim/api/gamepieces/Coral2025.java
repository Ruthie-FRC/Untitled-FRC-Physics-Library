package jsim.api.gamepieces;

import jsim.api.GamePieceState;
import jsim.api.GamePiecePhysics;
import edu.wpi.first.math.geometry.Pose3d;

/**
 * Implementation of 2025 Game Piece layout bounds.
 */
public class Coral2025 extends GamePieceState {

    /**
     * Creates a new Coral2025 game piece.
     */
    public Coral2025() {
        super(new GamePiecePhysics());
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
