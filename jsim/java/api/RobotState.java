package api;

import driver.WPILibClones.Pose2d;
import driver.WPILibClones.Translation2d;
import driver.WPILibClones.ChassisSpeeds;

public class RobotState {
    private RobotID id;
    private Pose2d pose;
    private Translation2d[] frameVertices;
    private ChassisSpeeds chassisSpeeds;

    public RobotState(RobotID id, Pose2d pose, Translation2d[] frameVertices) {
        this.id = id;
        this.pose = pose;
        this.frameVertices = frameVertices;
        this.chassisSpeeds = new ChassisSpeeds(0,0,0);
    }

    public Pose2d getPose() {
        return pose;
    }

    public void setPose(Pose2d pose) {
        this.pose = pose;
    }

    public Translation2d[] getFrameVertices() {
        return frameVertices;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return chassisSpeeds;
    }

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        this.chassisSpeeds = speeds;
    }
}