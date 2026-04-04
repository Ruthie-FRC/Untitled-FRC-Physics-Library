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

  /** Returns +X unit vector. */
  public static Vec3 unitX() {
    return new Vec3(1.0, 0.0, 0.0);
  }

  /** Returns +Y unit vector. */
  public static Vec3 unitY() {
    return new Vec3(0.0, 1.0, 0.0);
  }

  /** Returns +Z unit vector. */
  public static Vec3 unitZ() {
    return new Vec3(0.0, 0.0, 1.0);
  }

  /**
   * Adds another vector to this vector.
   *
   * @param other vector to add
   * @return summed vector
   */
  public Vec3 add(Vec3 other) {
    return new Vec3(x + other.x, y + other.y, z + other.z);
  }

  /**
   * Subtracts another vector from this vector.
   *
   * @param other vector to subtract
   * @return difference vector
   */
  public Vec3 subtract(Vec3 other) {
    return new Vec3(x - other.x, y - other.y, z - other.z);
  }

  /**
   * Scales this vector by a scalar value.
   *
   * @param scalar scalar multiplier
   * @return scaled vector
   */
  public Vec3 scale(double scalar) {
    return new Vec3(x * scalar, y * scalar, z * scalar);
  }

  /**
   * Divides this vector by scalar using C++ parity behavior.
   *
   * <p>When divisor is near zero, returns zero vector instead of NaN/Inf.
   */
  public Vec3 divide(double scalar) {
    if (Math.abs(scalar) <= Math.ulp(1.0)) {
      return ZERO;
    }
    return new Vec3(x / scalar, y / scalar, z / scalar);
  }

  /**
   * Computes the dot product with another vector.
   *
   * @param other vector to dot with
   * @return dot product value
   */
  public double dot(Vec3 other) {
    return x * other.x + y * other.y + z * other.z;
  }

  /**
   * Computes cross product with another vector.
   */
  public Vec3 cross(Vec3 other) {
    return new Vec3(
        (y * other.z) - (z * other.y),
        (z * other.x) - (x * other.z),
        (x * other.y) - (y * other.x));
  }

  /**
   * Computes squared magnitude of this vector.
   *
   * @return squared magnitude
   */
  public double normSquared() {
    return x * x + y * y + z * z;
  }

  /** Alias for C++ naming parity. */
  public double norm2() {
    return normSquared();
  }

  /**
   * Computes magnitude of this vector.
   *
   * @return Euclidean magnitude
   */
  public double norm() {
    return Math.sqrt(normSquared());
  }

  /**
   * Returns a normalized copy of this vector.
   *
   * @return unit-length vector or zero if input is near-zero
   */
  public Vec3 normalized() {
    double n = norm();
    return n > Math.ulp(1.0) ? divide(n) : ZERO;
  }

  /** Returns true when vector magnitude is near zero. */
  public boolean isZero(double epsilon) {
    return normSquared() < epsilon * epsilon;
  }

  /** Returns true when any component is NaN. */
  public boolean hasNaN() {
    return Double.isNaN(x) || Double.isNaN(y) || Double.isNaN(z);
  }

  /** Returns planar speed in XY plane. */
  public double planarSpeed() {
    return Math.hypot(x, y);
  }

  /** Returns XY components with z=0. */
  public Vec3 xy() {
    return new Vec3(x, y, 0.0);
  }

  /** Returns normalized direction in XY plane. */
  public Vec3 planarDir() {
    double mag = planarSpeed();
    return mag > Math.ulp(1.0) ? new Vec3(x / mag, y / mag, 0.0) : ZERO;
  }

  /** Projects this vector onto the provided axis vector. */
  public Vec3 projectOnto(Vec3 axis) {
    double denom = axis.normSquared();
    if (denom <= Math.ulp(1.0)) {
      return ZERO;
    }
    return axis.scale(dot(axis) / denom);
  }

  /** Reflects this vector about a plane normal. */
  public Vec3 reflect(Vec3 normal) {
    return subtract(normal.scale(2.0 * dot(normal)));
  }

  /** Convenience torque helper: returns r x F. */
  public Vec3 torqueAtOffset(Vec3 leverArmMeters) {
    return leverArmMeters.cross(this);
  }

  /** Linear interpolation between two vectors. */
  public static Vec3 lerp(Vec3 a, Vec3 b, double t) {
    return a.scale(1.0 - t).add(b.scale(t));
  }

  /** Euclidean distance between two vectors. */
  public static double distance(Vec3 a, Vec3 b) {
    return a.subtract(b).norm();
  }

  /** Per-component clamp. */
  public Vec3 clamp(Vec3 min, Vec3 max) {
    return new Vec3(
        Math.max(min.x, Math.min(x, max.x)),
        Math.max(min.y, Math.min(y, max.y)),
        Math.max(min.z, Math.min(z, max.z)));
  }
}