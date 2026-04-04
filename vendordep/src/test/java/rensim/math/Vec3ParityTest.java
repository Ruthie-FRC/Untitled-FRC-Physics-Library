package rensim.math;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import rensim.Vec3;

public class Vec3ParityTest {
  @Test
  void crossDotAndNormMatchVector3Expectations() {
    Vec3 a = new Vec3(1.0, 2.0, 3.0);
    Vec3 b = new Vec3(-4.0, 5.0, -6.0);

    Vec3 cross = a.cross(b);
    assertEquals(-27.0, cross.x(), 1.0e-12);
    assertEquals(-6.0, cross.y(), 1.0e-12);
    assertEquals(13.0, cross.z(), 1.0e-12);
    assertEquals(-12.0, a.dot(b), 1.0e-12);
    assertEquals(Math.sqrt(14.0), a.norm(), 1.0e-12);
  }

  @Test
  void projectionAndReflectionFollowCxxBehavior() {
    Vec3 v = new Vec3(3.0, 4.0, 0.0);
    Vec3 axis = Vec3.unitX();

    Vec3 projected = v.projectOnto(axis);
    assertEquals(3.0, projected.x(), 1.0e-12);
    assertEquals(0.0, projected.y(), 1.0e-12);

    Vec3 reflected = v.reflect(new Vec3(0.0, 1.0, 0.0));
    assertEquals(3.0, reflected.x(), 1.0e-12);
    assertEquals(-4.0, reflected.y(), 1.0e-12);
    assertEquals(0.0, reflected.z(), 1.0e-12);
  }

  @Test
  void zeroSafeDivisionAndPlanarHelpers() {
    Vec3 v = new Vec3(5.0, 0.0, 2.0);
    assertEquals(Vec3.ZERO, v.divide(0.0));

    Vec3 planar = v.xy();
    assertEquals(5.0, planar.x(), 1.0e-12);
    assertEquals(0.0, planar.y(), 1.0e-12);
    assertEquals(0.0, planar.z(), 1.0e-12);

    assertTrue(v.planarSpeed() > 0.0);
  }
}
