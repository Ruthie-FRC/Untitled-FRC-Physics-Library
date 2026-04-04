package frcsim_physics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frcsim_physics.RigidBody.Quaternion;
import org.junit.jupiter.api.Test;
import rensim.Vec3;

public class QuaternionParityTest {
  @Test
  void axisAngleRotationMatchesRightHandRule() {
    Quaternion q = Quaternion.fromAxisAngle(Vec3.unitZ(), Math.PI / 2.0);
    Vec3 rotated = q.rotate(Vec3.unitX());

    assertEquals(0.0, rotated.x(), 1.0e-9);
    assertEquals(1.0, rotated.y(), 1.0e-9);
    assertEquals(0.0, rotated.z(), 1.0e-9);
  }

  @Test
  void integrateUsesCxxAngularDerivativeShape() {
    Quaternion start = Quaternion.IDENTITY;
    Vec3 omega = new Vec3(0.0, 0.0, 2.0);
    double dt = 1.0e-3;

    Quaternion integrated = start.integrate(omega, dt);
    Quaternion expected = Quaternion.fromAxisAngle(Vec3.unitZ(), omega.z() * dt);

    assertEquals(expected.w(), integrated.w(), 2.0e-6);
    assertEquals(expected.x(), integrated.x(), 2.0e-6);
    assertEquals(expected.y(), integrated.y(), 2.0e-6);
    assertEquals(expected.z(), integrated.z(), 2.0e-6);
    assertTrue(Math.abs(integrated.norm() - 1.0) < 1.0e-9);
  }
}
