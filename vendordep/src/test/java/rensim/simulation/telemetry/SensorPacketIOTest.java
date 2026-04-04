package rensim.simulation.telemetry;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

public class SensorPacketIOTest {
  @Test
  void parseJsonLinesProducesTypedPackets() {
    String jsonl = """
        {"tick":1,"time_s":0.02,"contact_count":0,"bodies":[{"name":"robot","x_m":2.2,"y_m":4.0,"vx_mps":3.4,"vy_mps":0.0,"speed_mps":3.4}]}
        {"tick":2,"time_s":0.04,"contact_count":1,"bodies":[{"name":"robot","x_m":2.27,"y_m":4.0,"vx_mps":3.3,"vy_mps":0.0,"speed_mps":3.3},{"name":"note_a","x_m":6.0,"y_m":4.0,"vx_mps":-0.5,"vy_mps":0.0,"speed_mps":0.5}]}
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
            new BodyTelemetry("robot", 2.9, 4.2, 3.1, 0.0, 3.1),
            new BodyTelemetry("note_a", 6.1, 4.0, -0.2, 0.0, 0.2)));

    Map<String, Double> flat = SensorPacketIO.flattenForNetworkTables(packet);
    assertEquals(13, flat.size());
    assertEquals(7.0, flat.get("sim/tick"));
    assertEquals(0.14, flat.get("sim/time_s"), 1.0e-9);
    assertEquals(2.0, flat.get("sim/contact_count"));
    assertTrue(flat.containsKey("sim/body/0/x_m"));
    assertTrue(flat.containsKey("sim/body/1/speed_mps"));
  }
}
