// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

package rensim;

/**
 * Immutable 3D vector used for positions, velocities, and accelerations.
 *
 * @param x the x component
 * @param y the y component
 * @param z the z component
 */
public record Vec3(double x, double y, double z) {
  /** Zero vector with all components set to 0.0. */
  public static final Vec3 ZERO = new Vec3(0.0, 0.0, 0.0);
}
