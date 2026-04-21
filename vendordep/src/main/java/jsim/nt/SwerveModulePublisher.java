package jsim.nt;

import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import jsim.Vec3;

/**
 * Publishes swerve module states (position, velocity, angle) to NetworkTables for visualization.
 *
 * Topic structure:
 *   /jsim/swerve/modules/positions   (Nx3 array: [x, y, z, ...])
 *   /jsim/swerve/modules/velocities (Nx3 array: [vx, vy, vz, ...])
 *   /jsim/swerve/modules/angles     (N array: [angle0, angle1, ...])
 */
/**
 * Publishes swerve module states (position, velocity, angle) to NetworkTables for visualization.
 *
 * Topic structure:
 *   /jsim/swerve/modules/positions   (Nx3 array: [x, y, z, ...])
 *   /jsim/swerve/modules/velocities (Nx3 array: [vx, vy, vz, ...])
 *   /jsim/swerve/modules/angles     (N array: [angle0, angle1, ...])
 */
public class SwerveModulePublisher implements AutoCloseable {
    /** Publisher for swerve module positions (Nx3 array). */
    private final DoubleArrayPublisher positionsPublisher;
    /** Publisher for swerve module velocities (Nx3 array). */
    private final DoubleArrayPublisher velocitiesPublisher;
    /** Publisher for swerve module angles (N array). */
    private final DoubleArrayPublisher anglesPublisher;
    /** Number of swerve modules. */
    private final int numModules;

    /**
     * Constructs a SwerveModulePublisher with the default NetworkTableInstance and base topic.
     * @param numModules Number of swerve modules to publish.
     */
    public SwerveModulePublisher(int numModules) {
        this(numModules, NetworkTableInstance.getDefault(), "/jsim/swerve/modules");
    }

    /**
     * Constructs a SwerveModulePublisher with a custom NetworkTableInstance and base topic.
     * @param numModules Number of swerve modules to publish.
     * @param ntInstance NetworkTableInstance to use for publishing.
     * @param baseTopic Base topic path for module data.
     */
    public SwerveModulePublisher(int numModules, NetworkTableInstance ntInstance, String baseTopic) {
        this.numModules = numModules;
        NetworkTable table = ntInstance.getTable(baseTopic);
        this.positionsPublisher = table.getDoubleArrayTopic("positions").publish();
        this.velocitiesPublisher = table.getDoubleArrayTopic("velocities").publish();
        this.anglesPublisher = table.getDoubleArrayTopic("angles").publish();
    }

    /**
     * Publishes all swerve module states for this frame.
     *
     * @param positions Array of module positions (length N, each Vec3 is x/y/z in meters).
     * @param velocities Array of module velocities (length N, each Vec3 is vx/vy/vz in m/s).
     * @param angles Array of module angles in radians (length N).
     * @throws IllegalArgumentException if any array length does not match numModules.
     */
    public void publishFrame(Vec3[] positions, Vec3[] velocities, double[] angles) {
        if (positions.length != numModules || velocities.length != numModules || angles.length != numModules) {
            throw new IllegalArgumentException("Array lengths must match numModules");
        }
        double[] posFlat = new double[numModules * 3];
        double[] velFlat = new double[numModules * 3];
        for (int i = 0; i < numModules; ++i) {
            posFlat[i * 3] = positions[i].x();
            posFlat[i * 3 + 1] = positions[i].y();
            posFlat[i * 3 + 2] = positions[i].z();
            velFlat[i * 3] = velocities[i].x();
            velFlat[i * 3 + 1] = velocities[i].y();
            velFlat[i * 3 + 2] = velocities[i].z();
        }
        positionsPublisher.set(posFlat);
        velocitiesPublisher.set(velFlat);
        anglesPublisher.set(angles);
    }

    /**
     * Closes all NetworkTables publishers associated with this swerve module publisher.
     */
    @Override
    public void close() {
        positionsPublisher.close();
        velocitiesPublisher.close();
        anglesPublisher.close();
    }
}
