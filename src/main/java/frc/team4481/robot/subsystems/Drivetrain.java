package frc.team4481.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4481.lib.path.AdaptivePurePursuitController;
import frc.team4481.lib.subsystems.SubsystemBase;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.lib.swerve.*;
import frc.team4481.robot.motion.WeekendSlewRateLimiter;

import static frc.team4481.robot.Constants.HardwareMap.*;
import static frc.team4481.robot.Constants.Drivetrain.*;
import static frc.team4481.robot.Constants.Drivetrain.Comp.*;
import static frc.team4481.robot.Constants.*;
import static frc.team4481.robot.Constants.Field.*;

public class Drivetrain extends SubsystemBase<DrivetrainManager> {
    private final SubsystemHandler subsystemHandler = SubsystemHandler.getInstance();

    private final Translation2d frontLeftLocation =
            new Translation2d(DRIVETRAIN_WHEELBASE_DISTANCE, DRIVETRAIN_WHEELBASE_DISTANCE);
    private final Translation2d frontRightLocation =
            new Translation2d(DRIVETRAIN_WHEELBASE_DISTANCE, -DRIVETRAIN_WHEELBASE_DISTANCE);
    private final Translation2d backLeftLocation =
            new Translation2d(-DRIVETRAIN_WHEELBASE_DISTANCE, DRIVETRAIN_WHEELBASE_DISTANCE);
    private final Translation2d backRightLocation =
            new Translation2d(-DRIVETRAIN_WHEELBASE_DISTANCE, -DRIVETRAIN_WHEELBASE_DISTANCE);

    private SwerveModule frontLeft;
    private SwerveModule frontRight;
    private SwerveModule backLeft;
    private SwerveModule backRight;

    public final Pigeon2 pigeon = new Pigeon2(PIGEON_IMU);

    //Objects used for heading correction
    Timer timer = new Timer();
    double previousT;
    double offT;

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            frontLeftLocation,
            frontRightLocation,
            backLeftLocation,
            backRightLocation
    );

    private final SecondOrderSwerveKinematics secondKinematics = new SecondOrderSwerveKinematics(
            frontLeftLocation,
            frontRightLocation,
            backLeftLocation,
            backRightLocation
    );

    // Odometry
    private final SwerveDrivePoseEstimator poseEstimator;
    private final SwerveDriveOdometry odometry;

    //Skew rate limiter
    private WeekendSlewRateLimiter directionRateLimiter = new WeekendSlewRateLimiter(MAX_DIRECTION_RATE_LIMIT);

    // Pure Pursuit
    public final AdaptivePurePursuitController aPPController;
    public final Field2d field = new Field2d();

    private final SwerveDrivetrainHelper swerveDrivetrainHelper;

    public Drivetrain(){
        name = "Drivetrain";
        subsystemManager = new DrivetrainManager();

        aPPController = new AdaptivePurePursuitController(
                DRIVETRAIN_WIDTH,
                MAX_VELOCITY,
                MIN_VELOCITY,
                MAX_ACCELERATION,
                PATH_END_DISTANCE,
                MAX_LOOKAHEAD,
                MIN_LOOKAHEAD
        );

        initializeSwerveModules();

        if (DriverStation.getAlliance() == DriverStation.Alliance.Red){
            poseEstimator = new SwerveDrivePoseEstimator(
                    kinematics,
                    Rotation2d.fromDegrees(pigeon.getYaw()),
                    getSwerveModulePositions(),
                    DRIVETRAIN_START_POSITION_RED
            );
            subsystemManager.setTargetHeading(DRIVETRAIN_START_POSITION_RED.getRotation());
        } else {
            poseEstimator = new SwerveDrivePoseEstimator(
                    kinematics,
                    Rotation2d.fromDegrees(pigeon.getYaw()),
                    getSwerveModulePositions(),
                    DRIVETRAIN_START_POSITION_BLUE
            );
            subsystemManager.setTargetHeading(DRIVETRAIN_START_POSITION_BLUE.getRotation());
        }

        odometry = new SwerveDriveOdometry(
            kinematics,
            Rotation2d.fromDegrees(pigeon.getYaw()),
            getSwerveModulePositions(),
            new Pose2d()
        );

        subsystemManager.setAPPController(aPPController);
        subsystemManager.setPoseEstimator(poseEstimator);

        swerveDrivetrainHelper = new SwerveDrivetrainHelper(
                frontLeft,
                frontRight,
                backLeft,
                backRight,
                kinematics,
                secondKinematics,
                poseEstimator
        );
        swerveDrivetrainHelper.setMaxVelocity(MAX_VELOCITY);
        subsystemManager.setVelocityLimit(MAX_VELOCITY);
        swerveDrivetrainHelper.setFieldRelative(true);
    }

    @Override
    public void onStart(double timestamp) {
        if (DriverStation.isAutonomous()){
            //AUTONOMOUS IS HANDLED BY THE APP CONTROLLER, DO NOTHING HERE
            subsystemManager.setControlState(DrivetrainManager.controlState.DISABLED);
        } else {
            subsystemManager.setControlState(DrivetrainManager.controlState.ENABLED);
        }

        subsystemManager.setLocked(false);
        zeroSensors();


        subsystemManager.setDriveCurrentLimit(STALL_LIMIT_DRIVE);
        int limit = subsystemManager.getDriveCurrentLimit();
        frontLeft.setDriveCurrentLimit(limit);
        frontRight.setDriveCurrentLimit(limit);
        backLeft.setDriveCurrentLimit(limit);
        backRight.setDriveCurrentLimit(limit);
    }

    @Override
    public void readPeriodicInputs() {

    }

    @Override
    public void onLoop(double timestamp) {
        switch (subsystemManager.getControlState()) {
            case DISABLED:
                swerveDrivetrainHelper.idleSwerveModuleStates();
                break;
            case ENABLED:

                if (!subsystemManager.isCurrentLimitUpdated()){
                    int limit = subsystemManager.getDriveCurrentLimit();
                    frontLeft.setDriveCurrentLimit(limit);
                    frontRight.setDriveCurrentLimit(limit);
                    backLeft.setDriveCurrentLimit(limit);
                    backRight.setDriveCurrentLimit(limit);
                    subsystemManager.setCurrentLimitUpdated(true);
                }

                subsystemManager.setUseVision(checkVisionMeasurements(timestamp));

                subsystemManager.setCurrentPose(
                        poseEstimator.updateWithTime(
                                timestamp,
                                Rotation2d.fromDegrees(pigeon.getYaw()),
                                getSwerveModulePositions()
                        )
                );

                odometry.update(Rotation2d.fromDegrees(pigeon.getYaw()), getSwerveModulePositions());

//                NetworkTableInstance.getDefault().getTable("logtable").getEntry("poseestimatorpose").setDoubleArray(poseToArray(odometry.getPoseMeters()));

                field.setRobotPose(subsystemManager.getCurrentPose());
//                field.setRobotPose(odometry.getPoseMeters());

                swerveDrivetrainHelper.setMaxVelocity(subsystemManager.getVelocityLimit());

                ChassisSpeeds desiredSpeeds = subsystemManager.getDesiredSpeeds();

                //Correct heading when heading deviates when changing direction
                ChassisSpeeds correctedSpeeds = correctHeading(desiredSpeeds);

                //Limit the rate of change of the direction of the robot based on current speed
                ChassisSpeeds correctedLimitedSpeeds = limitDirectionChange(correctedSpeeds);

                if (subsystemManager.getLocked()) {
                    swerveDrivetrainHelper.idleSwerveModuleStates();
                } else {
                    swerveDrivetrainHelper.updateSwerveModuleStates(correctedLimitedSpeeds);
                }
                break;
        }

    }


    @Override
    public void writePeriodicOutputs() {

    }

    @Override
    public void onStop(double timestamp) {
        swerveDrivetrainHelper.idleSwerveModuleStates();
        terminate();
    }

    @Override
    public void zeroSensors() {
        timer.reset();
        timer.start();
        previousT = 0;
        offT = 0;
    }

    public void resetRotation(Rotation2d rotation){
        resetOdometry(new Pose2d(poseEstimator.getEstimatedPosition().getTranslation(), rotation));
        subsystemManager.setTargetHeading( rotation );
    }

    @Override
    public void terminate() {
        subsystemManager.setControlState(DrivetrainManager.controlState.DISABLED);
    }

    @Override
    public void outputData() {
        SmartDashboard.putNumber("DT/Robot Yaw", getYaw().getDegrees());
//        SmartDashboard.putNumber("DT/Pigeon Roll", pigeon.getRoll());
//        SmartDashboard.putNumber("DT/Pigeon Pitch", pigeon.getPitch());
        double[] sixdquad = new double[4];
        pigeon.get6dQuaternion(sixdquad);
        SmartDashboard.putNumberArray("DT/pigeon/6D quaternion", sixdquad);

        double[] pigeon_arr = new double[3];
        pigeon.getYawPitchRoll(pigeon_arr);
        SmartDashboard.putNumberArray("DT/pigeon/Yaw Pitch Roll", pigeon_arr);
        pigeon.getAccumGyro(pigeon_arr);
        SmartDashboard.putNumberArray("DT/pigeon/Accumulated Gyro", pigeon_arr);
        pigeon.getRawGyro(pigeon_arr);
        SmartDashboard.putNumberArray("DT/pigeon/Raw Gyro", pigeon_arr);

        short[] ba_xyz = new short[3];
        pigeon.getBiasedAccelerometer(ba_xyz);
        for (int i = 0; i < 3; i++) {
            pigeon_arr[i] = (double) (ba_xyz[i] / 16384.0 * 9.80665); //m/s^2
        }
        SmartDashboard.putNumberArray("DT/pigeon/Biased Accelerometer", pigeon_arr);

        

        SmartDashboard.putData(field);
        try {
            SmartDashboard.putString("DT/Current pose", subsystemManager.getCurrentPose().toString());

            SmartDashboard.putNumberArray("DT/Estimated pose array", poseToArray(subsystemManager.getCurrentPose()));
        } catch (Exception e) {}
        SmartDashboard.putBoolean("DT/vision/use vision", subsystemManager.getUseVision());
//        SmartDashboard.putBoolean("DT/vision/is locked", subsystemManager.getLocked());
        SmartDashboard.putNumber("DT/average wheel velocity", getAverageWheelVelocity());
        SmartDashboard.putNumber("DT/front back diff", getFrontBackDifferene());
        SmartDashboard.putString("DT/state", subsystemManager.getControlState().toString());
        SmartDashboard.putNumber("DT/target speed", Math.hypot(subsystemManager.getDesiredSpeeds().vxMetersPerSecond, subsystemManager.getDesiredSpeeds().vyMetersPerSecond));
        SmartDashboard.putNumber("DT/target heading", subsystemManager.getTargetHeading().getDegrees());

    }

    public void updateCurrentPose() {
        subsystemManager.setCurrentPose(poseEstimator.getEstimatedPosition());
    }

    /**
     * Set the target heading of the drivetrain such that it faces towards the
     * position that is input
     *
     * @param lookAtTrans {@code Translation2d} object that represents the position
     *                                         that the drivetrain will face
     */
    public void lookAtPosition(Translation2d lookAtTrans){
        Translation2d currentTrans =  subsystemManager.getCurrentPose().getTranslation();
        Translation2d deltaTrans = lookAtTrans.minus(currentTrans);

        subsystemManager.setTargetHeading (
                new Rotation2d( Math.atan2(deltaTrans.getY(), deltaTrans.getX()) )
        );
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose){
        subsystemManager.setCurrentPose(pose);

        poseEstimator.resetPosition(
                Rotation2d.fromDegrees(pigeon.getYaw()),
                getSwerveModulePositions(),
                pose
        );
    }

    /**
     * Gets the current positions of the swerve modules
     *
     * @return the current positions of the swerve modules
     */
    public SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[]{
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        };
    }

    /**
     * Initializes all the swerve modules on the drivetrain with the correct constants
     */
    private void initializeSwerveModules() {
        final PIDValueContainer TurnPIDValues = new PIDValueContainer(TURN_kP, TURN_kI, TURN_kD);
        final FFValueContainer TurnFFValues = new FFValueContainer(TURN_kS, TURN_kV, TURN_kA);
        final PIDValueContainer DrivePIDValues = new PIDValueContainer(DRIVE_kP, DRIVE_kI, DRIVE_kD);
        final FFValueContainer DriveFFValues = new FFValueContainer(DRIVE_kS, DRIVE_kV, DRIVE_kA);

        SwerveModule[] modules =  new SwerveModule[4];

        final int[] DRIVE_IDS =
                { DT_FRONT_LEFT_DRIVE_ID, DT_FRONT_RIGHT_DRIVE_ID, DT_BACK_LEFT_DRIVE_ID, DT_BACK_RIGHT_DRIVE_ID };
        final int[] TURN_IDS =
                { DT_FRONT_LEFT_TURN_ID, DT_FRONT_RIGHT_TURN_ID, DT_BACK_LEFT_TURN_ID, DT_BACK_RIGHT_TURN_ID };
        final boolean[] INVERTED =
                { DT_FRONT_LEFT_INVERTED, DT_FRONT_RIGHT_INVERTED, DT_BACK_LEFT_INVERTED, DT_BACK_RIGHT_INVERTED };

        for (int i = 0; i < modules.length; i++) {
            SwerveTurnHelper turnHelper = new SwerveTurnHelper(
                    TURN_IDS[i],
                    TurnPIDValues,
                    TurnFFValues,
                    STALL_LIMIT_TURN
            );
            SwerveDriveHelper driveHelper = new SwerveDriveHelper(
                    DRIVE_IDS[i],
                    INVERTED[i],
                    DRIVE_POSITION_CONVERSION_FACTOR,
                    DRIVE_VELOCITY_CONVERSION_FACTOR,
                    DrivePIDValues,
                    DriveFFValues,
                    STALL_LIMIT_DRIVE,
                    DRIVE_IDLE_MODE
            );

            modules[i] = new SwerveModule(turnHelper, driveHelper);
        }

        frontLeft = modules[0];
        frontRight = modules[1];
        backLeft = modules[2];
        backRight = modules[3];
    }

    /**
     * Optimizes the chassis speed that is put into the kinematics object to allow the robot to hold its heading
     * when no angular velocity is input.
     * The robot will therefore correct itself when it turns without telling it to do so.
     *
     * @param desiredSpeed desired chassis speed that is input by the controller
     * @return corrected {@code ChassisSpeeds} which takes into account that the robot needs to have the same heading
     * when no rotational speed is input
     */
    private ChassisSpeeds correctHeading(ChassisSpeeds desiredSpeed){

        //Determine time interval
        double currentT = timer.get();
        double dt = currentT - previousT;

        //Get desired rotational speed in radians per second and absolute translational speed in m/s
        double vr = desiredSpeed.omegaRadiansPerSecond;
        double v = Math.hypot(desiredSpeed.vxMetersPerSecond, desiredSpeed.vyMetersPerSecond);

        if (vr > 0.01 || vr < -0.01){
            offT = currentT;
            subsystemManager.setTargetHeading(getYaw());
            return desiredSpeed;
        }
        if (currentT - offT < 0.5){
            subsystemManager.setTargetHeading(getYaw());
            return desiredSpeed;
        }
        if (v < 0.1 && !subsystemManager.isTargetAligning()){
            subsystemManager.setTargetHeading(getYaw());
            return desiredSpeed;
        }

        //Determine target and current heading
        subsystemManager.setTargetHeading( subsystemManager.getTargetHeading().plus(new Rotation2d(vr * dt)) );
        Rotation2d currentHeading = getYaw();

        //Calculate the change in heading that is needed to achieve the target
        Rotation2d deltaHeading = subsystemManager.getTargetHeading().minus(currentHeading);

        if (Math.abs(deltaHeading.getDegrees()) < TURNING_DEADBAND){
            return desiredSpeed;
        }
        double correctedVr = deltaHeading.getRadians() / dt * HEADING_kP;

        previousT = currentT;

        return new ChassisSpeeds(desiredSpeed.vxMetersPerSecond, desiredSpeed.vyMetersPerSecond, correctedVr);
    }

    private ChassisSpeeds limitDirectionChange(ChassisSpeeds desiredSpeed){
        double vr = desiredSpeed.omegaRadiansPerSecond;
        double v = Math.hypot(desiredSpeed.vxMetersPerSecond, desiredSpeed.vyMetersPerSecond);
        double direction = Math.atan2(desiredSpeed.vyMetersPerSecond, desiredSpeed.vxMetersPerSecond);

        double rateLimit = ( MAX_VELOCITY / Math.abs(frontLeft.getDriveVelocity())) * MAX_DIRECTION_RATE_LIMIT * (Math.PI / 180);
        rateLimit = MathUtil.clamp(rateLimit, -1e3, 1e3);
        directionRateLimiter.setRateLimit(rateLimit);
        double limitedDirection = directionRateLimiter.calculate(direction);

        double vx = Math.cos(limitedDirection) * v;
        double vy = Math.sin(limitedDirection) * v;

        return new ChassisSpeeds(vx, vy, vr);
    }

    public void updateOdoWithVision() {
        double[] limelightPoseDouble = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpiblue").getDoubleArray(new double[]{0});
        Pose2d limelightPose = new Pose2d(new Translation2d(limelightPoseDouble[0], limelightPoseDouble[1]), Rotation2d.fromDegrees(limelightPoseDouble[5]));

        odometry.resetPosition(
                Rotation2d.fromDegrees(pigeon.getYaw()),
                getSwerveModulePositions(),
                limelightPose);
    }

    private boolean checkVisionMeasurements(double timestamp){
        try {
            double[] limelightPoseDoubleTop = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpiblue").getDoubleArray(new double[]{0});
            double[] limelightTargetSpacePoseDoubleTop = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_targetspace").getDoubleArray(new double[]{0});
            double tagAreaTop = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
            double[] limelightPoseDoubleBottom = NetworkTableInstance.getDefault().getTable("limelight-bottom").getEntry("botpose_wpiblue").getDoubleArray(new double[]{0});
            double[] limelightTargetSpacePoseDoubleBottom = NetworkTableInstance.getDefault().getTable("limelight-bottom").getEntry("botpose_targetspace").getDoubleArray(new double[]{0});
            double tagAreaBottom = NetworkTableInstance.getDefault().getTable("limelight-bottom").getEntry("ta").getDouble(0);

            //Check if the limelight rejects the position because it is too far away
            //This has to be done with the targetspace pose instead of the wpilibblue pose, because limelight still
            //updates the wpilibblue pose when it shouldn't update according to the pipeline settings
            double sumTop = 0;
            for (double value : limelightTargetSpacePoseDoubleTop) {
                sumTop += value;
            }
            boolean topInRange = sumTop != 0;
            double sumBottom = 0;
            for (double value : limelightTargetSpacePoseDoubleTop) {
                sumBottom += value;
            }
            boolean bottomInRange = sumBottom != 0;


            // Stop if limelight array is not full
            // Stop if translation X or Y is not in field
            // Stop if translation Z is negative
            double[] limelightPoseDouble;
            double tagArea;
            boolean topValid = topInRange &&
                    (limelightPoseDoubleTop.length > 5
                    && limelightPoseDoubleTop[0] > 0.75
                    && limelightPoseDoubleTop[0] < FIELD_LENGTH - 0.75
                    && limelightPoseDoubleTop[1] > 0.3
                    && limelightPoseDoubleTop[1] < FIELD_WIDTH - 0.3
                    && limelightPoseDoubleTop[2] >= 0
            );
            boolean bottomValid = bottomInRange &&
                    (limelightPoseDoubleBottom.length > 5
                    && limelightPoseDoubleBottom[0] > 0.75
                    && limelightPoseDoubleBottom[0] < FIELD_LENGTH - 0.75
                    && limelightPoseDoubleBottom[1] > 0.3
                    && limelightPoseDoubleBottom[1] < FIELD_WIDTH - 0.3
                    && limelightPoseDoubleBottom[2] >= 0);

            if (topValid && bottomValid){
                //Take the measurements from the top limelight, because that is the best
                limelightPoseDouble = limelightPoseDoubleTop;
                //Except for the x and y position, take the average of that
                limelightPoseDouble[0] = (limelightPoseDoubleBottom[0] + limelightPoseDoubleTop[0])/2;
                limelightPoseDouble[1] = (limelightPoseDoubleBottom[1] + limelightPoseDoubleTop[1])/2;
                //Take the tag area of the top limelight
                tagArea = tagAreaTop;
            } else if (bottomValid){
                limelightPoseDouble = limelightPoseDoubleBottom;
                tagArea = tagAreaBottom;
            } else if (topValid){
                limelightPoseDouble = limelightPoseDoubleTop;
                tagArea = tagAreaTop;
            } else {
                return false;
            }

            // Make pose from limelight translation and rotation
            // April tag rotation accuracy is lower than pigeon
//            Pose2d limelightPose = new Pose2d(new Translation2d(limelightPoseDouble[0], limelightPoseDouble[1]), Rotation2d.fromDegrees(pigeon.getYaw()));
            Pose2d limelightPose = new Pose2d(new Translation2d(limelightPoseDouble[0], limelightPoseDouble[1]), Rotation2d.fromDegrees(limelightPoseDouble[5]));
//            SmartDashboard.putString("DT/vision/limelight pose", limelightPose.toString());
//            SmartDashboard.putNumber("DT/vision/limelight yaw", limelightPoseDouble[5]);

//            NetworkTableInstance.getDefault().getTable("logtable").getEntry("limelightpose").setDoubleArray(poseToArray(limelightPose));
            SmartDashboard.putNumberArray("DT/vision/LL-top pose", new Double[]{limelightPoseDoubleTop[0], limelightPoseDoubleTop[1], limelightPoseDoubleTop[5]});
            SmartDashboard.putNumberArray("DT/vision/LL-bottom pose", new Double[]{limelightPoseDoubleBottom[0], limelightPoseDoubleBottom[1], limelightPoseDoubleBottom[5]});
            SmartDashboard.putNumberArray("DT/vision/LL final pose", new Double[]{limelightPoseDouble[0], limelightPoseDouble[1], limelightPoseDouble[5]});

            try {
                double[] limelightTagCorners = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tcornxy").getDoubleArray(new double[]{0});
                double[] limelightTagCornersX = new double[4];
                double[] limelightTagCornersY = new double[4];
                double[] limelightBottomTagCorners = NetworkTableInstance.getDefault().getTable("limelight-bottom").getEntry("tcornxy").getDoubleArray(new double[]{0});
                double[] limelightBottomTagCornersX = new double[4];
                double[] limelightBottomTagCornersY = new double[4];
                for (int i = 0; i < 4; i++) {
                    limelightTagCornersX[i] = limelightTagCorners[i * 2];
                    limelightTagCornersY[i] = limelightTagCorners[i * 2 + 1];

                    limelightBottomTagCornersX[i] = limelightBottomTagCorners[i * 2];
                    limelightBottomTagCornersY[i] = limelightBottomTagCorners[i * 2 + 1];
                }
                SmartDashboard.putNumberArray("DT/vision/LL-top tag corners x", limelightTagCornersX);
                SmartDashboard.putNumberArray("DT/vision/LL-top tag corners y", limelightTagCornersY);
                SmartDashboard.putNumberArray("DT/vision/LL-bottom tag corners x", limelightBottomTagCornersX);
                SmartDashboard.putNumberArray("DT/vision/LL-bottom tag corners y", limelightBottomTagCornersY);
            } catch (Exception e){}


            // Add estimator trust using april tag area
            double stdX = 0.13;
            double stdY = stdX;
            if (tagArea < 0.5){
                stdY *= 50;
                stdX *= 5;
            }
            poseEstimator.setVisionMeasurementStdDevs(new MatBuilder<>(Nat.N3(), Nat.N1()).fill(stdX, stdY, stdY*10));
            SmartDashboard.putNumber("DT/vision/april tag std X", stdX);
            SmartDashboard.putNumber("DT/vision/april tag std Y", stdY);

            // Add limelight latency
            double limelightLatency = limelightPoseDouble[6] / 1000;
            poseEstimator.addVisionMeasurement(limelightPose, timestamp - limelightLatency);

            return true;

        } catch (Exception e) {
            return false;
        }
    }

    private Rotation2d getYaw(){
        return poseEstimator.getEstimatedPosition().getRotation();
    }

    private double getAverageWheelVelocity(){
        return (Math.abs(frontLeft.getDriveVelocity()) + Math.abs(frontRight.getDriveVelocity()) + Math.abs(backLeft.getDriveVelocity()) + Math.abs(backRight.getDriveVelocity()))/4;
    }

    private double getFrontBackDifferene(){
        return (Math.abs(frontLeft.getDriveVelocity()) + Math.abs(frontRight.getDriveVelocity()))/2 - (Math.abs(backRight.getDriveVelocity()) + Math.abs(backLeft.getDriveVelocity()))/2;
    }

    private double[] poseToArray(Pose2d pose) {
        double[] arr = new double[3];
        arr[0] = pose.getX();
        arr[1] = pose.getY();
        arr[2] = pose.getRotation().getDegrees();

        return arr;
    }

}