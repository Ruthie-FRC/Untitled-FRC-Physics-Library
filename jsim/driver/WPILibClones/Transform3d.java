package driver.WPILibClones;

public class Transform3d {
    private double dx, dy, dz;
    private double droll, dpitch, dyaw;
    public Transform3d(double dx, double dy, double dz, double droll, double dpitch, double dyaw) {
        this.dx = dx;
        this.dy = dy;
        this.dz = dz;
        this.droll = droll;
        this.dpitch = dpitch;
        this.dyaw = dyaw;
    }
    public double getDx() { return dx; }
    public double getDy() { return dy; }
    public double getDz() { return dz; }
    public double getDRoll() { return droll; }
    public double getDPitch() { return dpitch; }
    public double getDYaw() { return dyaw; }
}