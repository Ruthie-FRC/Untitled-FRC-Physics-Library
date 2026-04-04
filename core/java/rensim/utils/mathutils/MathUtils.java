package rensim.utils.mathutils;

public final class MathUtils {
  private MathUtils() {}

  public static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }

  public static double lerp(double a, double b, double t) {
    return a + (b - a) * t;
  }

  public static double normalizeAngleRadians(double angle) {
    double a = angle;
    while (a > Math.PI) {
      a -= 2.0 * Math.PI;
    }
    while (a < -Math.PI) {
      a += 2.0 * Math.PI;
    }
    return a;
  }

  public static boolean epsilonEquals(double a, double b, double epsilon) {
    return Math.abs(a - b) <= Math.abs(epsilon);
  }
}