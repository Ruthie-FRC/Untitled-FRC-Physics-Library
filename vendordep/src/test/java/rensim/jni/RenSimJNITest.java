package rensim.jni;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import rensim.jni.RenSimJNI;

public class VendorJNITest {
  @Test
  void jniLinkTest() {
    RenSimJNI.initialize();

    long world = RenSimJNI.createWorld(0.01, true);
    assertTrue(world != 0);

    int body = RenSimJNI.createBody(world, 1.0);
    assertTrue(body >= 0);

    RenSimJNI.setBodyPosition(world, body, 0.0, 0.0, 1.0);
    RenSimJNI.stepWorld(world, 10);

    double[] pos = new double[3];
    int rc = RenSimJNI.getBodyPosition(world, body, pos);
    assertTrue(rc == 0);
    assertTrue(pos[2] < 1.0);

    RenSimJNI.destroyWorld(world);
  }
}
