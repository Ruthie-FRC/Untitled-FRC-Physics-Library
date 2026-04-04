package rensim.simulation;

import rensim.Vec3;

/**
 * Common interface for all simulated game pieces.
 */
public interface GamePiece {
  /**
   * Gets game-piece type identifier.
   *
   * @return game-piece type string
   */
  String type();

  /**
   * Gets current world pose as a lightweight pose record.
   *
   * @return current world pose
   */
  Pose3 pose();

  /**
   * Gets world velocity in meters per second.
   *
   * @return world velocity
   */
  Vec3 velocityMps();

  /**
   * Returns whether the piece is grounded in the arena.
   *
   * @return true for grounded game pieces
   */
  boolean grounded();

  /**
   * Callback triggered when game piece is considered scored/hit.
   */
  void triggerHitTargetCallback();
}