// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

package jsim.jni;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import jsim.PhysicsBody;
import jsim.PhysicsWorld;
import jsim.jni.JSimJNI;

public class JSimJNITest {
  @Test
  void jniLinkTest() {
    JSimJNI.initialize();

    long world = JSimJNI.createWorld(0.01, true);
    assertTrue(world != 0);

    int body = JSimJNI.createBody(world, 1.0);
    assertTrue(body >= 0);

    JSimJNI.setBodyPosition(world, body, 0.0, 0.0, 1.0);
    JSimJNI.stepWorld(world, 10);

    double[] pos = new double[3];
    int rc = JSimJNI.getBodyPosition(world, body, pos);
    assertTrue(rc == 0);
    assertTrue(pos[2] < 1.0);

    JSimJNI.destroyWorld(world);
  }

  @Test
  void javaWrapperScalarAndArrayTest() {
    try (PhysicsWorld world = new PhysicsWorld(0.01, true)) {
      PhysicsBody body = world.createBody(1.0);
      body.setPosition(0.0, 0.0, 1.0);
      body.setLinearVelocity(0.0, 0.0, 0.0);
      world.setGravity(0.0, 0.0, -9.81);

      world.step(10);

      double[] pos = body.positionArray();
      double[] vel = body.linearVelocityArray();
      assertTrue(pos.length == 3);
      assertTrue(vel.length == 3);
      assertTrue(pos[2] < 1.0);
      assertTrue(vel[2] < 0.0);
    }
  }
}
