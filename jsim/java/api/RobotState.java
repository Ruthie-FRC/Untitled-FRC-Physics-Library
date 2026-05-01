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
        this.chassisSpeeds = new ChassisSpeeds(0, 0, 0);
    }
}
