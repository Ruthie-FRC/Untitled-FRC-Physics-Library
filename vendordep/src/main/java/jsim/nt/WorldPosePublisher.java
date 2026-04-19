// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

package jsim.nt;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import jsim.jni.JSimJNI;

/**
 * Publishes JSim body state to NetworkTables in AdvantageScope-friendly formats.
 *
 * <p>Publishes both:
 * <ul>
 *   <li>Structured Pose3d array: {@code /jsim/world/bodyPoses}</li>
 *   <li>Flat full state [x,y,z,qw,qx,qy,qz,vx,vy,vz,wx,wy,wz...]: {@code /jsim/world/bodyState13Flat}</li>
 *   <li>Flat pose buffer [x,y,z,qw,qx,qy,qz...]: {@code /jsim/world/bodyPose7Flat}</li>
 *   <li>Flat velocity buffer [vx,vy,vz,wx,wy,wz...]: {@code /jsim/world/bodyVelocity6Flat}</li>
 * </ul>
 */
public class WorldPosePublisher implements AutoCloseable {
  private final long worldHandle;
  private final int maxBodies;

  private final StructArrayPublisher<Pose3d> bodyPosesPublisher;
  private final DoubleArrayPublisher bodyStateFlatPublisher;
  private final DoubleArrayPublisher bodyPoseFlatPublisher;
  private final DoubleArrayPublisher bodyVelocityFlatPublisher;

  private final double[] state13Buffer;

  /**
   * Creates a new world publisher under {@code /jsim/world}.
   *
   * @param worldHandle native world handle
   * @param maxBodies maximum number of bodies to export per frame
   */
  public WorldPosePublisher(long worldHandle, int maxBodies) {
    this(worldHandle, maxBodies, NetworkTableInstance.getDefault(), "/jsim/world");
  }

  /**
   * Creates a new world publisher.
   *
   * @param worldHandle native world handle
   * @param maxBodies maximum number of bodies to export per frame
   * @param ntInstance network table instance
   * @param baseTopic base topic path (e.g. {@code /jsim/world})
   */
  public WorldPosePublisher(
      long worldHandle,
      int maxBodies,
      NetworkTableInstance ntInstance,
      String baseTopic) {
    this.worldHandle = worldHandle;
    this.maxBodies = Math.max(1, maxBodies);
    this.state13Buffer = new double[this.maxBodies * 13];

    NetworkTable table = ntInstance.getTable(baseTopic);
    this.bodyPosesPublisher = table.getStructArrayTopic("bodyPoses", Pose3d.struct).publish();
    this.bodyStateFlatPublisher = table.getDoubleArrayTopic("bodyState13Flat").publish();
    this.bodyPoseFlatPublisher = table.getDoubleArrayTopic("bodyPose7Flat").publish();
    this.bodyVelocityFlatPublisher = table.getDoubleArrayTopic("bodyVelocity6Flat").publish();
  }

  /**
   * Pulls world state from JNI and publishes one frame.
   *
   * @return number of body entries published, or negative on native error
   */
  public int publishFrame() {
    int count = JSimJNI.getBodyState13Array(worldHandle, state13Buffer);
    if (count < 0) {
      return count;
    }

    Pose3d[] poses = new Pose3d[count];
    double[] poseFlat = new double[count * 7];
    double[] velocityFlat = new double[count * 6];
    for (int i = 0; i < count; i++) {
      int stateBase = i * 13;
      int poseBase = i * 7;
      int velBase = i * 6;

      poseFlat[poseBase] = state13Buffer[stateBase];
      poseFlat[poseBase + 1] = state13Buffer[stateBase + 1];
      poseFlat[poseBase + 2] = state13Buffer[stateBase + 2];
      poseFlat[poseBase + 3] = state13Buffer[stateBase + 3];
      poseFlat[poseBase + 4] = state13Buffer[stateBase + 4];
      poseFlat[poseBase + 5] = state13Buffer[stateBase + 5];
      poseFlat[poseBase + 6] = state13Buffer[stateBase + 6];

      velocityFlat[velBase] = state13Buffer[stateBase + 7];
      velocityFlat[velBase + 1] = state13Buffer[stateBase + 8];
      velocityFlat[velBase + 2] = state13Buffer[stateBase + 9];
      velocityFlat[velBase + 3] = state13Buffer[stateBase + 10];
      velocityFlat[velBase + 4] = state13Buffer[stateBase + 11];
      velocityFlat[velBase + 5] = state13Buffer[stateBase + 12];

      Translation3d translation =
          new Translation3d(poseFlat[poseBase], poseFlat[poseBase + 1], poseFlat[poseBase + 2]);
      Rotation3d rotation =
          new Rotation3d(
              new Quaternion(
                  poseFlat[poseBase + 3],
                  poseFlat[poseBase + 4],
                  poseFlat[poseBase + 5],
                  poseFlat[poseBase + 6]));
      poses[i] = new Pose3d(translation, rotation);
    }

    bodyPosesPublisher.set(poses);
    double[] stateFlat = new double[count * 13];
    System.arraycopy(state13Buffer, 0, stateFlat, 0, stateFlat.length);
    bodyStateFlatPublisher.set(stateFlat);
    bodyPoseFlatPublisher.set(poseFlat);
    bodyVelocityFlatPublisher.set(velocityFlat);

    return count;
  }

  @Override
  public void close() {
    bodyPosesPublisher.close();
    bodyStateFlatPublisher.close();
    bodyPoseFlatPublisher.close();
    bodyVelocityFlatPublisher.close();
  }
}
