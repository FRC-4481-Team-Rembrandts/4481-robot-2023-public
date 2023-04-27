package frc.team4481.robot.auto.actions;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.team4481.lib.auto.actions.Action;
import frc.team4481.lib.path.TrajectoryHandler;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.robot.subsystems.Drivetrain;
import frc.team4481.robot.subsystems.DrivetrainManager;

import java.util.List;

public class WaitForMarkerAction implements Action {

    private final Drivetrain drivetrain;
    private final DrivetrainManager drivetrainManager;

    private TrajectoryHandler trajectoryHandler = TrajectoryHandler.getInstance();

    private PathPlannerTrajectory.EventMarker marker;

    private final String markerName;
    private final double triggerDistance;

    /**
     * Action that waits until the robot is closer than the trigger distance to the specified event marker.
     *
     * @param markerName the name of the event marker to target
     * @param triggerDistance the euclidean distance from the target to the center of the robot at which to trigger.
     */
    public WaitForMarkerAction(String markerName, double triggerDistance) {
        SubsystemHandler subsystemHandler = SubsystemHandler.getInstance();
        drivetrain = (Drivetrain) subsystemHandler.getSubsystemByClass(Drivetrain.class);
        drivetrainManager = drivetrain.getSubsystemManager();

        this.triggerDistance = triggerDistance;
        this.markerName = markerName;
    }

    @Override
    public void start() {
        while (marker == null) {
            try {
                marker = getMarkerWithName(
                        trajectoryHandler.getCurrentTrajectory().getMarkers(),
                        markerName
                );
            } catch (NullPointerException e) {
                DataLogManager.log("path not loaded yet");
            }
        }

    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return drivetrainManager.getCurrentPose().getTranslation().getDistance(marker.positionMeters) < triggerDistance;
    }

    @Override
    public void done() {
        DataLogManager.log("Marker found: " + markerName);
    }

    private PathPlannerTrajectory.EventMarker getMarkerWithName(
            List<PathPlannerTrajectory.EventMarker> markers,
            String name
    ) {
        for (PathPlannerTrajectory.EventMarker marker : markers) {
            for (String markerName : marker.names) {
                if (markerName.equals(name)) {
                    return marker;
                }
            }
        }
        return null;
    }
}
