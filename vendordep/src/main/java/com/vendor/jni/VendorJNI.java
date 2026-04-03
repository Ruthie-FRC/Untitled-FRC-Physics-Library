package com.vendor.jni;

import java.util.concurrent.atomic.AtomicBoolean;

/**
 * JNI entry points for the vendor physics driver.
 */
public class VendorJNI {
  private VendorJNI() {}

  static boolean libraryLoaded = false;

  /**
   * Configures whether the native library is loaded during static initialization.
   */
  public static class Helper {
    private Helper() {}

    private static AtomicBoolean extractOnStaticLoad = new AtomicBoolean(true);

    /**
     * Returns whether the driver loads during static initialization.
     *
     * @return true when the driver loads on static initialization
     */
    public static boolean getExtractOnStaticLoad() {
      return extractOnStaticLoad.get();
    }

    /**
     * Sets whether the driver loads during static initialization.
     *
     * @param load the new value
     */
    public static void setExtractOnStaticLoad(boolean load) {
      extractOnStaticLoad.set(load);
    }
  }

  static {
    if (Helper.getExtractOnStaticLoad()) {
      System.loadLibrary("VendorDriver");
      libraryLoaded = true;
    }
  }

  /**
   * Forces the native library to load.
   */
  public static synchronized void forceLoad() {
    if (libraryLoaded) {
      return;
    }
    System.loadLibrary("VendorDriver");
    libraryLoaded = true;
  }

  /**
   * Initializes the native driver.
   *
   * @return the value returned by the driver
   */
  public static native int initialize();

  /**
   * Creates a native world handle.
   *
   * @param fixedDtSeconds the fixed simulation timestep in seconds
   * @param enableGravity true to enable gravity for the world
   * @return the native world handle
   */
  public static native long createWorld(double fixedDtSeconds, boolean enableGravity);

  /**
   * Destroys a native world handle.
   *
   * @param worldHandle the native world handle to destroy
   */
  public static native void destroyWorld(long worldHandle);

  /**
   * Creates a body in the given world and returns its native index.
   *
   * @param worldHandle the native world handle
   * @param massKg the body mass in kilograms
   * @return the native body index
   */
  public static native int createBody(long worldHandle, double massKg);

  /**
   * Sets a body's position in meters.
   *
   * @param worldHandle the native world handle
   * @param bodyIndex the native body index
   * @param xMeters the x position in meters
   * @param yMeters the y position in meters
   * @param zMeters the z position in meters
   * @return zero on success
   */
  public static native int setBodyPosition(long worldHandle, int bodyIndex, double xMeters, double yMeters,
      double zMeters);

  /**
   * Sets a body's linear velocity in meters per second.
   *
   * @param worldHandle the native world handle
   * @param bodyIndex the native body index
   * @param vxMps the x velocity in meters per second
   * @param vyMps the y velocity in meters per second
   * @param vzMps the z velocity in meters per second
   * @return zero on success
   */
  public static native int setBodyLinearVelocity(long worldHandle, int bodyIndex, double vxMps, double vyMps,
      double vzMps);

  /**
   * Enables or disables gravity for a body.
   *
   * @param worldHandle the native world handle
   * @param bodyIndex the native body index
   * @param enabled true to enable gravity, false to disable it
   * @return zero on success
   */
  public static native int setBodyGravityEnabled(long worldHandle, int bodyIndex, boolean enabled);

  /**
   * Sets the world's gravity vector in meters per second squared.
   *
   * @param worldHandle the native world handle
   * @param gxMps2 the x gravity component in meters per second squared
   * @param gyMps2 the y gravity component in meters per second squared
   * @param gzMps2 the z gravity component in meters per second squared
   * @return zero on success
   */
  public static native int setWorldGravity(long worldHandle, double gxMps2, double gyMps2, double gzMps2);

  /**
   * Advances the world by the given number of steps.
   *
   * @param worldHandle the native world handle
   * @param steps the number of steps to advance
   * @return zero on success
   */
  public static native int stepWorld(long worldHandle, int steps);

  /**
   * Reads a body's position into {@code outXyzMeters}.
   *
   * @param worldHandle the native world handle
   * @param bodyIndex the native body index
   * @param outXyzMeters the output array that receives the position
   * @return zero on success
   */
  public static native int getBodyPosition(long worldHandle, int bodyIndex, double[] outXyzMeters);

  /**
   * Reads a body's linear velocity into {@code outVxyzMps}.
   *
   * @param worldHandle the native world handle
   * @param bodyIndex the native body index
   * @param outVxyzMps the output array that receives the linear velocity
   * @return zero on success
   */
  public static native int getBodyLinearVelocity(long worldHandle, int bodyIndex, double[] outVxyzMps);
}
