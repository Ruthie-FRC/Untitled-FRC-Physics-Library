package rensim.simulation.cad;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import rensim.Vec3;

/**
 * Lightweight CAD import reader for JSON manifests exported by cad-import scripts.
 */
public final class CadImportService {
  private static final Pattern STRING_VALUE = Pattern.compile("\"%s\"\\s*:\\s*\"([^\"]*)\"");
  private static final Pattern NUMBER_VALUE = Pattern.compile("\"%s\"\\s*:\\s*([-+]?\\d*\\.?\\d+(?:[eE][-+]?\\d+)?)");

  private CadImportService() {}

  public static CadImportResult importFromManifest(Path manifestPath) {
    try {
      String json = Files.readString(manifestPath);
      String modelName = readString(json, "modelName");
      String sourcePath = readString(json, "sourcePath");
      double bx = readNumber(json, "boundsX");
      double by = readNumber(json, "boundsY");
      double bz = readNumber(json, "boundsZ");

      List<RobotCadModel.RobotCadLink> links = parseLinks(json);
      RobotCadModel model = new RobotCadModel(modelName, sourcePath, links, new Vec3(bx, by, bz));
      return new CadImportResult(true, "Imported CAD manifest", model);
    } catch (IOException e) {
      return new CadImportResult(false, "Failed to read CAD manifest: " + e.getMessage(), null);
    } catch (IllegalArgumentException e) {
      return new CadImportResult(false, "Invalid CAD manifest: " + e.getMessage(), null);
    }
  }

  private static List<RobotCadModel.RobotCadLink> parseLinks(String json) {
    List<RobotCadModel.RobotCadLink> links = new ArrayList<>();
    Pattern linkPattern = Pattern.compile("\\{\\s*\"name\"\\s*:\\s*\"([^\"]+)\"\\s*,\\s*\"massKg\"\\s*:\\s*([-+]?\\d*\\.?\\d+(?:[eE][-+]?\\d+)?)\\s*,\\s*\"comX\"\\s*:\\s*([-+]?\\d*\\.?\\d+(?:[eE][-+]?\\d+)?)\\s*,\\s*\"comY\"\\s*:\\s*([-+]?\\d*\\.?\\d+(?:[eE][-+]?\\d+)?)\\s*,\\s*\"comZ\"\\s*:\\s*([-+]?\\d*\\.?\\d+(?:[eE][-+]?\\d+)?)\\s*\\}");
    Matcher matcher = linkPattern.matcher(json);
    while (matcher.find()) {
      String name = matcher.group(1);
      double massKg = Double.parseDouble(matcher.group(2));
      double comX = Double.parseDouble(matcher.group(3));
      double comY = Double.parseDouble(matcher.group(4));
      double comZ = Double.parseDouble(matcher.group(5));
      links.add(new RobotCadModel.RobotCadLink(name, massKg, new Vec3(comX, comY, comZ)));
    }
    return links;
  }

  private static String readString(String json, String key) {
    Matcher m = Pattern.compile(String.format(STRING_VALUE.pattern(), Pattern.quote(key))).matcher(json);
    if (!m.find()) {
      throw new IllegalArgumentException("Missing key: " + key);
    }
    return m.group(1);
  }

  private static double readNumber(String json, String key) {
    Matcher m = Pattern.compile(String.format(NUMBER_VALUE.pattern(), Pattern.quote(key))).matcher(json);
    if (!m.find()) {
      throw new IllegalArgumentException("Missing key: " + key);
    }
    return Double.parseDouble(m.group(1));
  }
}
