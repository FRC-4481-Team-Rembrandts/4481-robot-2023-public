package frc.team4481.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import frc.team4481.lib.path.AdaptivePurePursuitController;
import frc.team4481.lib.subsystems.SubsystemManagerBase;

public class DrivetrainManager extends SubsystemManagerBase {

    private ChassisSpeeds desiredSpeeds = new ChassisSpeeds(0,0,0);
    private controlState currentControlState = controlState.DISABLED;

    private AdaptivePurePursuitController aPPController;
    private SwerveDriveOdometry odometry;
    private SwerveDrivePoseEstimator poseEstimator;
    private Pose2d currentPose;
    private Rotation2d targetHeading;
    private double velocityLimit;
    private boolean useVision;
    private boolean locked = false;
    private boolean isTargetAligning = false;
    private int driveCurrentLimit;
    private boolean currentLimitUpdated = false;

    public enum controlState {
        DISABLED,
        ENABLED
    }

    public ChassisSpeeds getDesiredSpeeds() {
        return desiredSpeeds;
    }
    public void setDesiredSpeeds(ChassisSpeeds desiredSpeeds) {
        this.desiredSpeeds = desiredSpeeds;
    }

    public void setControlState(controlState pControlState) {
        currentControlState = pControlState;
    }
    public controlState getControlState() {
        return currentControlState;
    }

    public void setAPPController(AdaptivePurePursuitController aPPController) {
        this.aPPController = aPPController;
    }
    public AdaptivePurePursuitController getAPPController() {
        return aPPController;
    }

    public void setPoseEstimator(SwerveDrivePoseEstimator poseEstimator){
        this.poseEstimator = poseEstimator;
    }
    public SwerveDrivePoseEstimator getPoseEstimator() { return poseEstimator; }

    public Pose2d getCurrentPose() {
        return currentPose;
    }
    public void setCurrentPose(Pose2d currentPose) {
        this.currentPose = currentPose;
    }

    public Rotation2d getTargetHeading(){ return targetHeading; }
    public void setTargetHeading(Rotation2d targetHeading) { this.targetHeading = targetHeading; }

    public double getVelocityLimit() {return velocityLimit;}
    public void setVelocityLimit(double targetVelocity) {this.velocityLimit = targetVelocity;}

    public boolean getUseVision() {
        return useVision;
    }
    public void setUseVision(boolean useVision) {
        this.useVision = useVision;
    }

    public boolean getLocked() {
        return locked;
    }
    public void setLocked(boolean locked) {
        this.locked = locked;
    }
    public boolean isTargetAligning() {
        return isTargetAligning;
    }

    public void setTargetAligning(boolean targetAligning) {
        isTargetAligning = targetAligning;
    }

    public int getDriveCurrentLimit() {
        return driveCurrentLimit;
    }

    public void setDriveCurrentLimit(int driveCurrentLimit) {
        this.driveCurrentLimit = driveCurrentLimit;
        setCurrentLimitUpdated(false);
    }

    public boolean isCurrentLimitUpdated() {
        return currentLimitUpdated;
    }

    public void setCurrentLimitUpdated(boolean currentLimitUpdated) {
        this.currentLimitUpdated = currentLimitUpdated;
    }


}
