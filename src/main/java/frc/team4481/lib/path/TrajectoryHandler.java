package frc.team4481.lib.path;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

import java.util.ArrayList;
import java.util.HashMap;

import static frc.team4481.robot.Constants.*;

/**
 * Utility class to load and transform a {@code Trajectory} such that it can be used by an
 * {@code AdaptivePurePursuitController}. This class might be useful for other tools as well
 * but these are not these are currently not supported.
 */
public class TrajectoryHandler {
    private static TrajectoryHandler instance;

    private PathPlannerTrajectory currentTrajectory;
    private final HashMap<String, PathPlannerTrajectory> loadedTrajectoryMap = new HashMap<>();

    public boolean hasInitialPose = false;

    private TrajectoryHandler() {}

    /**
     * Gets the {@code TrajectoryHandler} instance.
     *
     * @return singleton instance of the {@code TrajectoryHandler}
     */
    public static TrajectoryHandler getInstance() {
        if (instance == null)
            instance = new TrajectoryHandler();

        return instance;
    }

    /**
     * Preloads a Path Planner 2 path for faster processing.
     * This method is intended to be called in the disabled state of the robot.
     *
     * @param pathName      Filename of the path minus file extension
     * @param isReversed  Should the robot follow the path in reverse
     * @param maxVelocity Maximum robot velocity on the path in m/s
     * @param maxAcceleration Maximum robot acceleration on the path in m/s^2
     */
    public void preloadTrajectory(
            String pathName,
            boolean isReversed,
            double maxVelocity,
            double maxAcceleration
    ) {
        loadedTrajectoryMap.put(
                pathName,
                PathPlanner.loadPath(pathName, maxVelocity, maxAcceleration, isReversed)
        );
    }

    /**
     * Preloads a Path Planner 2 path for faster processing.
     * This method is intended to be called in the disabled state of the robot.
     *
     * @param pathName      Filename of the path minus file extension
     * @param maxVelocity Maximum robot velocity on the path in m/s
     * @param maxAcceleration Maximum robot acceleration on the path in m/s^2
     */
    public void preloadTrajectory(
            String pathName,
            double maxVelocity,
            double maxAcceleration
    ) {
        preloadTrajectory(pathName, false, maxVelocity, maxAcceleration);
    }

    /**
     * Preloads a Path Planner 2 path for faster processing.
     * This method is intended to be called in the disabled state of the robot.
     *
     * @param pathName Filename of the path minus file extension
     */
    public void preloadTrajectory(String pathName) {
        preloadTrajectory(pathName, MAX_VELOCITY, MAX_ACCELERATION);
    }

    /**
     * Gets the current Path Planner Trajectory.
     *
     * @return current Path Planner trajectory
     */
    public PathPlannerTrajectory getCurrentTrajectory() {
        return currentTrajectory;
    }

    /**
     * Constructs a {@code List} of {@code PathPlannerStates} from the current trajectory.
     *
     * @return {@code ArrayList} of states in the trajectory.
     */
    public ArrayList<PathPlannerTrajectory.PathPlannerState> getStatesFromTrajectory() {
        return getStatesFromTrajectory(currentTrajectory);
    }

    /**
     * Transforms a Path Planner 2 path into a {@code PathPlannerTrajectory}. Preloading a path increases loading speed
     *
     * @param pathName      Filename of the path minus file extension
     * @param isReversed  Should the robot follow the path in reverse
     * @param maxVelocity Maximum robot velocity on the path in m/s
     * @param maxAcceleration Maximum robot acceleration on the path in m/s^2
     *
     * @see TrajectoryHandler#preloadTrajectory(String, boolean, double, double) preloading trajectories
     */
    public PathPlannerTrajectory setTrajectory(
            String pathName,
            boolean isReversed,
            double maxVelocity,
            double maxAcceleration
    ) {
        try {
            if (loadedTrajectoryMap.containsKey(pathName))
                return setPreloadedTrajectory(pathName);
            return setUnloadedTrajectory(pathName, isReversed, maxVelocity, maxAcceleration);
        } catch (PathNotLoadedException e) {
            e.printStackTrace();
        }

        return null;
    }

    /**
     * Linearly extrapolates the given trajectory with a given length and velocity setpoints of 0.
     *
     * @param original original trajectory from which to extrapolate
     * @param length length of the extrapolated trajectory
     * @return the original trajectory extrapolated with the given length
     */
    public ArrayList<PathPlannerTrajectory.PathPlannerState> extrapolateStates(
            ArrayList<PathPlannerTrajectory.PathPlannerState> original,
            double length
    ) {
        PathPlannerTrajectory.PathPlannerState initialState = original.get(original.size() - 1);
        Pose2d initialPose = initialState.poseMeters;

//        Transform2d extrapolation = new Transform2d(
//                new Translation2d(length, new Rotation2d()),
//                new Rotation2d()
//        );

        Twist2d extrapolation = new Twist2d(length, 0, 0);

        Pose2d endPose = initialPose.exp(extrapolation);

        PathPoint initialPoint = new PathPoint(initialPose.getTranslation(), initialPose.getRotation(), initialState.holonomicRotation);
        PathPoint endPoint = new PathPoint(endPose.getTranslation(), endPose.getRotation(), initialState.holonomicRotation);

        PathPlannerTrajectory trajectory = PathPlanner.generatePath(
                new PathConstraints(0,0),
                initialPoint,
                endPoint
        );

        original.addAll(getStatesFromTrajectory(trajectory));

        return original;
    }

    /**
     * Sets a preloaded {@code PathPlannerTrajectory} as current trajectory.
     *
     * @param pathName name of the path as noted in Path Planner 2
     * @throws PathNotLoadedException if the path has not been loaded yet
     * @return the currently loaded trajectory
     */
    private PathPlannerTrajectory setPreloadedTrajectory(String pathName) throws PathNotLoadedException {
        if (!loadedTrajectoryMap.containsKey(pathName))
            throw new PathNotLoadedException("Path '" + pathName + "' has not been preloaded yet.");

        currentTrajectory = loadedTrajectoryMap.get(pathName);

        return currentTrajectory;
    }

    /**
     * Sets an unloaded {@code PathPlannerTrajectory} as current trajectory and loads it for future use.
     *
     * @param pathName name of the path as noted in Path Planner 2
     * @throws PathNotLoadedException if the path has not been loaded yet
     * @return the currently loaded trajectory
     */
    private PathPlannerTrajectory setUnloadedTrajectory(
            String pathName,
            boolean isReversed,
            double maxVelocity,
            double maxAcceleration
    ) throws PathNotLoadedException {
        preloadTrajectory(pathName, isReversed, maxVelocity, maxAcceleration);

        return setPreloadedTrajectory(pathName);
    }

    /**
     * Constructs a {@code List} of {@code PathPlannerStates} from the current trajectory.
     *
     * @return {@code ArrayList} of states in the trajectory.
     */
    private ArrayList<PathPlannerTrajectory.PathPlannerState> getStatesFromTrajectory(PathPlannerTrajectory trajectory) {
        // Prune the first entry since it has a target velocity of 0
        // This would result in a robot that would never move
        int i = 1;
        ArrayList<PathPlannerTrajectory.PathPlannerState> states = new ArrayList<>();

        // Add the rest of the states to a new arrayList
        do {
            states.add(trajectory.getState(i));
            i++;
        } while (!trajectory.getState(i).equals(trajectory.getEndState()));

        return states;
    }
}
