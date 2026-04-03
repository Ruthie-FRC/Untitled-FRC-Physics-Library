package com.vendor.jni;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

public class VendorJNITest {
  @Test
  void jniLinkTest() {
    VendorJNI.initialize();

    long world = VendorJNI.createWorld(0.01, true);
    assertTrue(world != 0);

    int body = VendorJNI.createBody(world, 1.0);
    assertTrue(body >= 0);

    VendorJNI.setBodyPosition(world, body, 0.0, 0.0, 1.0);
    VendorJNI.stepWorld(world, 10);

    double[] pos = new double[3];
    int rc = VendorJNI.getBodyPosition(world, body, pos);
    assertTrue(rc == 0);
    assertTrue(pos[2] < 1.0);

    VendorJNI.destroyWorld(world);
  }
}