package rensim.simulation.telemetry;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Utility methods for parsing runtime telemetry JSONL and producing NT-style scalar maps.
 */
public final class SensorPacketIO {
  private static final ObjectMapper MAPPER = new ObjectMapper();

  private SensorPacketIO() {}

  /**
   * Parses simulation telemetry JSONL from disk.
   *
   * @param jsonlPath path to JSON Lines telemetry file
   * @return immutable list of parsed packets
   */
  public static List<SensorPacket> readJsonLines(Path jsonlPath) {
    try {
      return parseJsonLines(Files.readString(jsonlPath));
    } catch (IOException ex) {
      throw new IllegalStateException("Failed to read telemetry JSONL: " + jsonlPath, ex);
    }
  }

  /**
   * Parses simulation telemetry JSONL from a raw string.
   *
   * @param jsonl raw JSON Lines content
   * @return immutable list of parsed packets
   */
  public static List<SensorPacket> parseJsonLines(String jsonl) {
    List<SensorPacket> packets = new ArrayList<>();
    for (String line : jsonl.split("\\R")) {
      String trimmed = line.trim();
      if (trimmed.isEmpty()) {
        continue;
      }
      packets.add(parsePacket(trimmed));
    }
    return List.copyOf(packets);
  }

  /**
   * Converts one packet into NetworkTables-style scalar key/value telemetry.
   *
   * @param packet sensor packet to flatten
   * @return ordered map with scalar telemetry values
   */
  public static Map<String, Double> flattenForNetworkTables(SensorPacket packet) {
    Map<String, Double> out = new LinkedHashMap<>();
    out.put("sim/tick", (double) packet.tick());
    out.put("sim/time_s", packet.timeSeconds());
    out.put("sim/contact_count", (double) packet.contactCount());

    List<BodyTelemetry> bodies = packet.bodies();
    for (int i = 0; i < bodies.size(); i++) {
      BodyTelemetry body = bodies.get(i);
      String base = "sim/body/" + i + "/";
      out.put(base + "x_m", body.xMeters());
      out.put(base + "y_m", body.yMeters());
      out.put(base + "vx_mps", body.vxMps());
      out.put(base + "vy_mps", body.vyMps());
      out.put(base + "speed_mps", body.speedMps());
    }
    return out;
  }

  private static SensorPacket parsePacket(String json) {
    JsonNode root;
    try {
      root = MAPPER.readTree(json);
    } catch (IOException ex) {
      throw new IllegalArgumentException("Invalid telemetry packet JSON", ex);
    }

    int tick = required(root, "tick").asInt();
    double timeSeconds = required(root, "time_s").asDouble();
    int contactCount = required(root, "contact_count").asInt();

    JsonNode bodyArray = required(root, "bodies");
    if (!bodyArray.isArray()) {
      throw new IllegalArgumentException("bodies must be an array");
    }

    List<BodyTelemetry> bodies = new ArrayList<>();
    for (JsonNode node : bodyArray) {
      bodies.add(new BodyTelemetry(
          required(node, "name").asText(),
          required(node, "x_m").asDouble(),
          required(node, "y_m").asDouble(),
          required(node, "vx_mps").asDouble(),
          required(node, "vy_mps").asDouble(),
          required(node, "speed_mps").asDouble()));
    }

    return new SensorPacket(tick, timeSeconds, contactCount, bodies);
  }

  private static JsonNode required(JsonNode root, String fieldName) {
    JsonNode value = root.get(fieldName);
    if (value == null) {
      throw new IllegalArgumentException("Missing required telemetry field: " + fieldName);
    }
    return value;
  }
}
