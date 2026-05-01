package api;

/**
 * Native Vector3 backed by C++ core (JNI).
 */
public class Vector3 {
    private long nativePtr;

    /**
     * Constructs a native vector.
     * @param x X value
     * @param y Y value
     * @param z Z value
     */
    public Vector3(double x, double y, double z) {
        nativePtr = nativeCreate(x, y, z);
    }

    /**
     * Internal JNI constructor from pointer.
     */
    Vector3(long ptr) {
        this.nativePtr = ptr;
    }

    /**
     * Returns the native pointer (for JNI use).
     */
    long getNativePtr() {
        return nativePtr;
    }

    /**
     * Returns the norm (magnitude) of the vector.
     */
    public double norm() {
        return nativeNorm(nativePtr);
    }

    /**
     * Releases native memory.
     */
    public void dispose() {
        nativeDelete(nativePtr);
        nativePtr = 0;
    }

    private native long nativeCreate(double x, double y, double z);
    private native double nativeNorm(long ptr);
    private native void nativeDelete(long ptr);

    static {
        System.loadLibrary("jsimcore");
    }
}
