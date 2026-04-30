package api;

public class RobotState {
    public Pose2d pose;
    public final Translation2d[] frameVertices;
    public ChassisSpeeds chassisSpeeds;
    public final RobotID id;

    public RobotState(RobotID id, Pose2d pose, Translation2d[] frameVertices) {
        this.id = id;
        this.pose = pose;
        this.frameVertices = frameVertices;
        this.chassisSpeeds = new ChassisSpeeds(0,0,0);
    }

    public Pose2d getPose(RobotID id) {
        return pose;
    }

    public void setPose(Pose2d pose, RobotID id) {
        this.pose = pose;
    }

    public Translation2d[] getFrameVertices(RobotID id) {
        return frameVertices;
    }

    public ChassisSpeeds getChassisSpeeds(RobotID id) {
        return chassisSpeeds;
    }

    public void setChassisSpeeds(ChassisSpeeds speeds, RobotID id) {
        this.chassisSpeeds = speeds;
    }
}