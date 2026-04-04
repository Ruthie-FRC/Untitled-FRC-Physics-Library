package rensim.math;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import frcsim_physics.RigidBody.Quaternion;
import java.io.IOException;
import java.io.InputStream;
import org.junit.jupiter.api.Test;
import rensim.Vec3;

public class MathContractFixtureTest {
  private static final ObjectMapper MAPPER = new ObjectMapper();

  @Test
  void vec3AndQuaternionMatchCppMathContractFixture() throws IOException {
    InputStream stream = getClass().getResourceAsStream("/math/cpp_math_contract.json");
    if (stream == null) {
      throw new IOException("Missing C++ math contract fixture resource");
    }

    JsonNode root = MAPPER.readTree(stream);
    double eps = root.get("epsilon").asDouble();

    JsonNode vector = root.get("vector");
    Vec3 a = toVec3(vector.get("a"));
    Vec3 b = toVec3(vector.get("b"));

    assertEquals(vector.get("dot").asDouble(), a.dot(b), eps);
    assertVec3Equals(toVec3(vector.get("cross")), a.cross(b), eps);
    assertEquals(vector.get("norm_a").asDouble(), a.norm(), eps);
    assertVec3Equals(toVec3(vector.get("normalized_a")), a.normalized(), eps);

    JsonNode quat = root.get("quaternion");
    Vec3 axis = toVec3(quat.get("axis"));
    double angle = quat.get("angle_rad").asDouble();
    Quaternion q = Quaternion.fromAxisAngle(axis, angle);

    JsonNode expectedQ = quat.get("from_axis_angle");
    assertEquals(expectedQ.get(0).asDouble(), q.w(), eps);
    assertEquals(expectedQ.get(1).asDouble(), q.x(), eps);
    assertEquals(expectedQ.get(2).asDouble(), q.y(), eps);
    assertEquals(expectedQ.get(3).asDouble(), q.z(), eps);

    assertVec3Equals(toVec3(quat.get("rotate_unit_x")), q.rotate(Vec3.unitX()), 1.0e-8);
    assertTrue(Math.abs(q.norm() - 1.0) < 1.0e-9);
  }

  private static Vec3 toVec3(JsonNode arr) {
    return new Vec3(arr.get(0).asDouble(), arr.get(1).asDouble(), arr.get(2).asDouble());
  }

  private static void assertVec3Equals(Vec3 expected, Vec3 actual, double eps) {
    assertEquals(expected.x(), actual.x(), eps);
    assertEquals(expected.y(), actual.y(), eps);
    assertEquals(expected.z(), actual.z(), eps);
  }
}
