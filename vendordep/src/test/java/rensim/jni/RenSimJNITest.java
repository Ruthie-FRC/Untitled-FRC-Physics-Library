// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

package rensim.jni;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import rensim.jni.JSimJNI;

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
}
