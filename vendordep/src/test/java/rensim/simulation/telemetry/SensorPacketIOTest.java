package rensim.simulation.telemetry;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

public class SensorPacketIOTest {
  @Test
  void parseJsonLinesProducesTypedPackets() {
    String jsonl = """
        {"tick":1,"time_s":0.02,"contact_count":0,"bodies":[{"name":"robot","x_m":2.2,"y_m":4.0,"position_frame_tag":"w","vx_mps":3.4,"vy_mps":0.0,"speed_mps":3.4,"velocity_frame_tag":"w"}]}
        {"tick":2,"time_s":0.04,"contact_count":1,"bodies":[{"name":"robot","x_m":2.27,"y_m":4.0,"position_frame_tag":"w","vx_mps":3.3,"vy_mps":0.0,"speed_mps":3.3,"velocity_frame_tag":"w"},{"name":"note_a","x_m":6.0,"y_m":4.0,"position_frame_tag":"w","vx_mps":-0.5,"vy_mps":0.0,"speed_mps":0.5,"velocity_frame_tag":"w"}]}
        """;

    List<SensorPacket> packets = SensorPacketIO.parseJsonLines(jsonl);
    assertEquals(2, packets.size());

    SensorPacket first = packets.get(0);
    assertEquals(1, first.tick());
    assertEquals(0.02, first.timeSeconds(), 1.0e-9);
    assertEquals(1, first.bodies().size());
    assertEquals("robot", first.bodies().get(0).name());
  }

  @Test
  void flattenForNetworkTablesMatchesRuntimeKeyLayout() {
    SensorPacket packet = new SensorPacket(
        7,
        0.14,
        2,
        List.of(
        new BodyTelemetry("robot", 2.9, 4.2, FrameTag.WORLD, 3.1, 0.0, 3.1, FrameTag.WORLD),
        new BodyTelemetry("note_a", 6.1, 4.0, FrameTag.WORLD, -0.2, 0.0, 0.2, FrameTag.WORLD)));

    Map<String, Double> flat = SensorPacketIO.flattenForNetworkTables(packet);
    assertEquals(13, flat.size());
    assertEquals(7.0, flat.get("sim/tick"));
    assertEquals(0.14, flat.get("sim/time_s"), 1.0e-9);
    assertEquals(2.0, flat.get("sim/contact_count"));
    assertTrue(flat.containsKey("sim/body/0/x_m"));
    assertTrue(flat.containsKey("sim/body/1/speed_mps"));
  }

  @Test
  void flattenRejectsBodyFrameData() {
    SensorPacket packet = new SensorPacket(
        3,
        0.06,
        0,
        List.of(new BodyTelemetry("robot", 1.0, 2.0, FrameTag.BODY, 0.1, 0.0, 0.1, FrameTag.WORLD)));

    assertThrows(IllegalArgumentException.class, () -> SensorPacketIO.flattenForNetworkTables(packet));
  }

  @Test
  void parseRejectsInvalidFrameTag() {
    String json =
        "{" +
        "\"tick\":1," +
        "\"time_s\":0.02," +
        "\"contact_count\":0," +
        "\"bodies\":[{" +
        "\"name\":\"robot\"," +
        "\"x_m\":1.0," +
        "\"y_m\":2.0," +
        "\"position_frame_tag\":\"x\"," +
        "\"vx_mps\":0.0," +
        "\"vy_mps\":0.0," +
        "\"speed_mps\":0.0," +
        "\"velocity_frame_tag\":\"w\"}]}";

    assertThrows(IllegalArgumentException.class, () -> SensorPacketIO.parseJsonLines(json));
  }

  @Test
  void goldenFixtureParsesAndFlattens() throws IOException {
    InputStream stream = getClass().getResourceAsStream("/telemetry/golden_packets.jsonl");
    assertNotNull(stream, "golden telemetry fixture is missing from test resources");

    String jsonl = new String(stream.readAllBytes(), StandardCharsets.UTF_8);
    List<SensorPacket> packets = SensorPacketIO.parseJsonLines(jsonl);
    assertEquals(2, packets.size());

    Map<String, Double> flatFirst = SensorPacketIO.flattenForNetworkTables(packets.get(0));
    Map<String, Double> flatLast = SensorPacketIO.flattenForNetworkTables(packets.get(1));

    assertEquals(1.0, flatFirst.get("sim/tick"));
    assertEquals(2.0, flatLast.get("sim/tick"));
    assertEquals(1.0, flatLast.get("sim/contact_count"));
    assertTrue(flatLast.containsKey("sim/body/1/speed_mps"));
  }
}
