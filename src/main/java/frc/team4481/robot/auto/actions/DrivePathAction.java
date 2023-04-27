package frc.team4481.robot.auto.actions;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4481.lib.auto.actions.Action;
import frc.team4481.lib.path.AdaptivePurePursuitController;
import frc.team4481.lib.path.TrajectoryHandler;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.robot.subsystems.Drivetrain;
import frc.team4481.robot.subsystems.DrivetrainManager;

import java.util.ArrayList;
import java.util.List;

/**
 * Action to drive a predefined path using the Pure Pursuit algorithm.
 */
public class DrivePathAction implements Action {
    private final String trajectory;
    private boolean reversed = false;
    private double maxVelocity = -1;
    private double maxAcceleration = -1;
    private boolean finishLocked = false;

    Pose2d currentPose;
    Pose2d targetPose;

    private final AdaptivePurePursuitController aPPController;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Drivetrain drivetrain;
    private final DrivetrainManager drivetrainManager;

    TrajectoryHandler trajectoryHandler = TrajectoryHandler.getInstance();

    /**
     * Create a new {@code DrivePathAction} that will follow a {@code Trajectory} using the Pure Pursuit algorithm.
     *
     * @param trajectory Trajectory that the robot has to follow. Use the filename without extension from Path Planner
     */
    public DrivePathAction(String trajectory){
        this.trajectory = trajectory;

        SubsystemHandler subsystemHandler = SubsystemHandler.getInstance();
        drivetrain = (Drivetrain) subsystemHandler.getSubsystemByClass(Drivetrain.class);
        drivetrainManager = drivetrain.getSubsystemManager();

        poseEstimator = drivetrainManager.getPoseEstimator();
        aPPController = drivetrainManager.getAPPController();
    }

    /**
     * Create a new {@code DrivePathAction} that will follow a {@code Trajectory} using the Pure Pursuit algorithm.
     *
     * @param trajectory Trajectory that the robot has to follow. Use the filename without extension from Path Planner
     * @param reversed Whether the path should be followed in reverse. For holonomic drivetrains, this is always false
     */
    public DrivePathAction(String trajectory, boolean reversed){
        this(trajectory);

        this.reversed = reversed;
    }

    /**
     * Create a new {@code DrivePathAction} that will follow a {@code Trajectory} using the Pure Pursuit algorithm.
     *
     * @param trajectory Trajectory that the robot has to follow. Use the filename without extension from Path Planner
     * @param reversed Whether the path should be followed in reverse
     */
    public DrivePathAction(String trajectory, boolean reversed, double maxVelocity, double maxAcceleration) {
        this(trajectory, reversed);

        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
    }

    public DrivePathAction(
            String trajectory, boolean reversed, double maxVelocity, double maxAcceleration, boolean finishLocked
    ) {
        this(trajectory, reversed, maxVelocity, maxAcceleration);

        this.finishLocked = finishLocked;
    }

    @Override
    public void start() {
        DataLogManager.log("Drive Action Start");

        Trajectory currentTrajectory;

        if(maxVelocity != -1 && maxAcceleration != -1) {
            currentTrajectory = aPPController.setNewTrajectory(trajectory, reversed, maxVelocity, maxAcceleration);
        } else {
            currentTrajectory = aPPController.setNewTrajectory(trajectory, reversed);
        }

        // Display trajectory on Driver Station
        drivetrain.field.getObject("trajectory").setTrajectory(currentTrajectory);

        ArrayList<PathPlannerTrajectory.PathPlannerState> states = aPPController.getTrajectoryStates();

        double[] poseArray = new double[states.size()*3];

        for (int i = 0; i < states.size(); i++) {
            PathPlannerTrajectory.PathPlannerState state = states.get(i);
            poseArray[i * 3] = state.poseMeters.getX();
            poseArray[i * 3 + 1] = state.poseMeters.getY();
            poseArray[i * 3 + 2] = state.holonomicRotation.getDegrees();
        }

        SmartDashboard.putString("Auto/trajectory name", trajectory);
        SmartDashboard.putNumberArray("Auto/trajectory", poseArray);

        // Move initial robot pose to first pose in current trajectory
        if(!trajectoryHandler.hasInitialPose) {
            // Calculate initial pose
            PathPlannerTrajectory.PathPlannerState initialState = trajectoryHandler.getStatesFromTrajectory().get(0);

            Translation2d initialPosition = initialState.poseMeters.getTranslation();
            Rotation2d initialRotation = initialState.holonomicRotation;
            Pose2d initialPose = new Pose2d(initialPosition, initialRotation);

            poseEstimator.resetPosition(
                    Rotation2d.fromDegrees(drivetrain.pigeon.getYaw()),
                    drivetrain.getSwerveModulePositions(),
                    initialPose
            );
            drivetrainManager.setTargetHeading(poseEstimator.getEstimatedPosition().getRotation());
            drivetrain.updateCurrentPose();

            trajectoryHandler.hasInitialPose = true;
        }

        // Tell drivetrain to follow the path
        drivetrainManager.setLocked(false);
        drivetrainManager.setControlState(DrivetrainManager.controlState.ENABLED);
    }

    @Override
    public void update() {
        currentPose = drivetrainManager.getCurrentPose();
        targetPose = aPPController.getTargetPose2d(currentPose);

        double targetVelocity = aPPController.getTargetVelocityForCurrentPose(currentPose);

        drivetrainManager.setDesiredSpeeds(
                aPPController.getChassisSpeedsForTargetDelta(currentPose, targetPose, targetVelocity)
        );

        drivetrain.field.setRobotPose(currentPose);
        drivetrain.field.getObject("lookahead").setPose(targetPose);
    }

    @Override
    public boolean isFinished() {
        return aPPController.isPathFinished();
    }

    @Override
    public void done() {
        drivetrainManager.setControlState(DrivetrainManager.controlState.DISABLED);
        drivetrain.field.getObject("trajectory").setTrajectory(new Trajectory());
        DataLogManager.log("Drive Action Done");

    }
}
