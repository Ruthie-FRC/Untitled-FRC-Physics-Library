package 
driver.WPILibClones;

public class Pose3d {
    private double x, y, z;
    private double roll, pitch, yaw;
    public Pose3d(double x, double y, double z, double roll, double pitch, double yaw) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.roll = roll;
        this.pitch = pitch;
        this.yaw = yaw;
    }
    public double getX() { return x; }
    public double getY() { return y; }
    public double getZ() { return z; }
    public double getRoll() { return roll; }
    public double getPitch() { return pitch; }
    public double getYaw() { return yaw; }
}