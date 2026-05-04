package jsim.field;

/**
 * Single JSON per season config parser mapping.
 */
public class FieldConfig {
    /** The game season associated with this field configuration. */
    public String season;
    /** Array of field elements in this configuration. */
    public FieldElement[] fieldElements;

    /**
     * Creates a new empty FieldConfig with no field elements.
     */
    public FieldConfig() {
        this.fieldElements = new FieldElement[0];
    }
}
