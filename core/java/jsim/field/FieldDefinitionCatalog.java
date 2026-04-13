// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

package jsim.field;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.util.Collections;
import java.util.Set;

/**
 * Built-in field definition catalog for vendordep runtime usage.
 *
 * <p>This allows teams to load standard season field JSON directly from the vendordep without
 * running external Python scripts.
 */
public final class FieldDefinitionCatalog {
  private static final ObjectMapper MAPPER = new ObjectMapper();
  private static final Set<Integer> BUILTIN_YEARS = Set.of(2024, 2025);

  private FieldDefinitionCatalog() {}

  /**
   * Returns available built-in season years bundled with the vendordep.
   *
   * @return immutable set of years
   */
  public static Set<Integer> availableYears() {
    return Collections.unmodifiableSet(BUILTIN_YEARS);
  }

  /**
   * Loads a built-in field definition JSON for a season year.
   *
   * @param year season year, e.g. 2024
   * @return JSON text for the requested field definition
   */
  public static String loadFieldJson(int year) {
    String resourcePath = toResourcePath(year);
    try (InputStream stream = FieldDefinitionCatalog.class.getResourceAsStream(resourcePath)) {
      if (stream == null) {
        throw new IllegalArgumentException("No built-in field definition for year " + year);
      }
      byte[] bytes = stream.readAllBytes();
      return new String(bytes, StandardCharsets.UTF_8);
    } catch (IOException e) {
      throw new IllegalStateException("Failed to read built-in field definition for year " + year, e);
    }
  }

  /**
   * Loads a built-in field definition as a parsed JsonNode.
   *
   * @param year season year, e.g. 2024
   * @return parsed JSON tree
   */
  public static JsonNode loadFieldNode(int year) {
    try {
      return MAPPER.readTree(loadFieldJson(year));
    } catch (IOException e) {
      throw new IllegalStateException("Failed to parse built-in field definition for year " + year, e);
    }
  }

  private static String toResourcePath(int year) {
    return "/jsim/fields/field_" + year + ".json";
  }
}
