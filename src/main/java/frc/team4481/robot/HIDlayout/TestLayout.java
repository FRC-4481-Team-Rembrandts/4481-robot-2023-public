package frc.team4481.robot.HIDlayout;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4481.lib.controller.ControlDevice;
import frc.team4481.lib.hid.HIDLayout;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.lib.throwable.HardwareException;
import frc.team4481.lib.util.CountingDelay;
import frc.team4481.robot.path.AutoAligner;
import frc.team4481.robot.subsystems.*;
import frc.team4481.robot.subsystems.Drivetrain;
import frc.team4481.robot.subsystems.Gripper;
import frc.team4481.robot.subsystems.Pneumatics;
import frc.team4481.robot.subsystems.Intake;
import frc.team4481.robot.subsystems.output.BlinkinLED;
import frc.team4481.robot.util.GamePieceHandler;

import static frc.team4481.lib.controller.IPS4HID.Axis.*;
import static frc.team4481.lib.controller.IPS4HID.Button.*;
import static frc.team4481.lib.controller.IPS4HID.DpadButton.*;
import static frc.team4481.robot.Constants.*;
import static frc.team4481.robot.Constants.Drivetrain.*;
import static frc.team4481.robot.Constants.MAX_VELOCITY;
import static frc.team4481.robot.util.GamePieceHandler.GamePiece.*;

public class TestLayout extends HIDLayout {
    private final SubsystemHandler subsystemHandler = SubsystemHandler.getInstance();
    private Drivetrain drivetrain;
    private DrivetrainManager drivetrainManager;
    private Gripper gripper;
    private GripperManager gripperManager;
    private Arm arm;
    private ArmManager armManager;
//    private Spindexer spindexer;
//    private SpindexerManager spindexerManager;
    private Intake intake;
    private IntakeManager intakeManager;
    private Pneumatics pneumatics;
    private PneumaticsManager pneumaticsManager;
    private final GamePieceHandler gamePieceHandler = GamePieceHandler.getInstance();
    private boolean previousCurrentBoostButton = false;
    private boolean gripperReleased = false;
    private double wallAngle;
    private boolean goWallAngle = false;
    private SlewRateLimiter driveSpeedSlewRateLimiter = new SlewRateLimiter(8);
    private boolean humanPlayerAutoAlignPossible = false;

    public TestLayout(ControlDevice driver, ControlDevice operator) {
        super(driver, operator);


    }

    @Override
    public void getSubsystemManagers() {
        drivetrain = (Drivetrain) subsystemHandler.getSubsystemByClass(Drivetrain.class);
        drivetrainManager = drivetrain.getSubsystemManager();
        arm = (Arm) subsystemHandler.getSubsystemByClass(Arm.class);
        armManager = arm.getSubsystemManager();
        gripper = (Gripper) subsystemHandler.getSubsystemByClass(Gripper.class);
        gripperManager = gripper.getSubsystemManager();
//        spindexer = (Spindexer) subsystemHandler.getSubsystemByClass(Spindexer.class);
//        spindexerManager = spindexer.getSubsystemManager();
        intake = (Intake) subsystemHandler.getSubsystemByClass(Intake.class);
        intakeManager = intake.getSubsystemManager();
        pneumatics = (Pneumatics) subsystemHandler.getSubsystemByClass(Pneumatics.class);
        pneumaticsManager = pneumatics.getSubsystemManager();

    }

    @Override
    public void updateOrange() throws HardwareException {

        // Trigger spindexer when intake is used
//        if(spindexerManager.getControlState() == SpindexerManager.ControlState.DISABLED && intakeManager.getControlState() == IntakeManager.controlState.INTAKE) {
//            spindexerManager.setControlState(SpindexerManager.ControlState.ENABLED);
//        }

        boolean crossPressed = driver.getRawButtonPressed(CROSS.id);

        if(crossPressed && !goWallAngle){
            if(Math.abs(drivetrainManager.getPoseEstimator().getEstimatedPosition().getRotation().getDegrees()) > 90){
//                drivetrainManager.setTargetHeading(Rotation2d.fromDegrees(0));
                wallAngle = 0;

            } else  {
//                drivetrainManager.setTargetHeading(Rotation2d.fromDegrees(180));\
                wallAngle = 180;
            }
            goWallAngle = true;
        } else if(crossPressed || Math.abs(driver.getAxisValue(RIGHTSTICK_X) )> 0.1) {
            goWallAngle = false;
            drivetrainManager.setTargetAligning(false);
            drivetrainManager.setTargetHeading(drivetrainManager.getCurrentPose().getRotation());
        }
        if(goWallAngle){
            drivetrainManager.setTargetAligning(true);
            drivetrainManager.setTargetHeading(Rotation2d.fromDegrees(wallAngle));
            if((Math.abs(drivetrainManager.getPoseEstimator().getEstimatedPosition().getRotation().getDegrees()) > wallAngle - WALL_ANGLE_MARGIN && wallAngle == 180) || (Math.abs(drivetrainManager.getPoseEstimator().getEstimatedPosition().getRotation().getDegrees()) < WALL_ANGLE_MARGIN && wallAngle == 0 )){
                goWallAngle = false;
                drivetrainManager.setTargetAligning(false);
            }
        }

        if (driver.getButtonValue(BUMPER_L1)) {
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("snapshot").setNumber(1);
            NetworkTableInstance.getDefault().getTable("limelight-bottom").getEntry("snapshot").setNumber(1);
            Pose2d currentPose = drivetrainManager.getPoseEstimator().getEstimatedPosition();
            boolean onGridSide;
            if (DriverStation.getAlliance() == DriverStation.Alliance.Blue){
                onGridSide = currentPose.getX() < Field.FIELD_LENGTH/2 - 2;
            } else {
                onGridSide = currentPose.getX() > Field.FIELD_LENGTH/2 + 2;
            }
            if (onGridSide){
                autoAlignGrid();
                drivetrainManager.setTargetAligning(false);
            } else {
                autoAlignKroket();
                double velocityMultiplier = drivetrainManager.getVelocityLimit();
//                setDrivetrainSpeed(
//                        filterDeadband(-driver.getAxisValue(LEFTSTICK_Y) * velocityMultiplier, ORANGE_LEFTSTICK_DEADBAND),
//                        filterDeadband(-driver.getAxisValue(LEFTSTICK_X) * velocityMultiplier, ORANGE_LEFTSTICK_DEADBAND),
//                        filterDeadband(-driver.getAxisValue(RIGHTSTICK_X) * velocityMultiplier, ORANGE_RIGHTSTICK_DEADBAND)
//                );
            }
        } else {
            humanPlayerAutoAlignPossible = false;
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("snapshot").setNumber(0);
            NetworkTableInstance.getDefault().getTable("limelight-bottom").getEntry("snapshot").setNumber(0);
//            double velocityMultiplier = MAX_VELOCITY - ( (MAX_VELOCITY - MIN_MAX_VELOCITY) * driver.getAxisValue(TRIGGER_L2));
//
//            drivetrainManager.setVelocityLimit(velocityMultiplier);
//            setDrivetrainSpeed(
//                    filterDeadband(-driver.getAxisValue(LEFTSTICK_Y) * velocityMultiplier, ORANGE_LEFTSTICK_DEADBAND),
//                    filterDeadband(-driver.getAxisValue(LEFTSTICK_X) * velocityMultiplier, ORANGE_LEFTSTICK_DEADBAND),
//                    filterDeadband(-driver.getAxisValue(RIGHTSTICK_X) * velocityMultiplier, ORANGE_RIGHTSTICK_DEADBAND)
//            );
            setDrivetrainSpeed(
                    filterDeadband(curveAxis(-driver.getAxisValue(LEFTSTICK_Y)) * MAX_VELOCITY, ORANGE_LEFTSTICK_DEADBAND),
                    filterDeadband(curveAxis(-driver.getAxisValue(LEFTSTICK_X)) * MAX_VELOCITY, ORANGE_LEFTSTICK_DEADBAND),
                    filterDeadband(-driver.getAxisValue(RIGHTSTICK_X) * MAX_VELOCITY, ORANGE_RIGHTSTICK_DEADBAND)
            );
        }

//        zeroDrivetrainRotation(driver.getButtonValue(OPTIONS) || driver.getButtonValue(SHARE));
        zeroDrivetrainRotation(driver.getButtonValue(OPTIONS));
//        drivetrainLookAtPosition(driver.getButtonValue(CIRCLE));

        if (driver.getButtonValue(BUMPER_R1)) {
            intakeManager.setControlState(IntakeManager.ControlState.INTAKE);
            if (gripperManager.getControlState() != GripperManager.ControlState.HOLD) {
//                armManager.setCurrentPositionState(ArmManager.PositionState.CUBE_INTAKE);
//                gripperManager.setControlState(GripperManager.ControlState.INTAKE);
            } else {
                armManager.setCurrentPositionState(ArmManager.PositionState.SPINDEXER_RETRACTED);
            }

        } else if (driver.getAxisValue(TRIGGER_R2)>0.1) {
            // TODO test if intake should keep rolling if pistons are retracting
            intakeManager.setControlState(IntakeManager.ControlState.REVERSE);
//            spindexerManager.setControlState(SpindexerManager.ControlState.REVERSE_FEEDER);
//        } else if (intakeManager.getControlState() == IntakeManager.ControlState.DISABLED && !intakeManager.getIsOccupied()){
//            intakeManager.setControlState(IntakeManager.ControlState.ROLLING);
        }
        else{
            intakeManager.setControlState(IntakeManager.ControlState.DISABLED);
            if (armManager.getCurrentPositionState() == ArmManager.PositionState.CUBE_INTAKE) {
                armManager.setCurrentPositionState(ArmManager.PositionState.SPINDEXER_RETRACTED);
                gripperManager.setControlState(GripperManager.ControlState.HOLD);
            }
        }
//
//        if (spindexerManager.getControlState() == SpindexerManager.ControlState.REVERSE_FEEDER && (driver.getAxisValue(TRIGGER_R2) < 0.1)){
//            spindexerManager.setControlState(SpindexerManager.ControlState.DISABLED);
//        }
//
//        if (driver.getButtonValue(CROSS)){
//            drivetrainManager.setTargetAligning(true);
//            drivetrainManager.setTargetHeading(Rotation2d.fromDegrees(180));
//            drivetrainManager.setSlowMode(true);
//        } else {
//            drivetrainManager.setTargetAligning(false);
//            drivetrainManager.setSlowMode(false);
//        }

        if (drivetrainManager.getDriveCurrentLimit() == STALL_LIMIT_DRIVE_CHARGE){
            pneumaticsManager.setLedControl(PneumaticsManager.LedControl.MANUAL);
            if (gamePieceHandler.getGamePiece() == CUBE){
                pneumaticsManager.setPattern(BlinkinLED.Pattern.C1_STROBE);
            } else {
                pneumaticsManager.setPattern(BlinkinLED.Pattern.C2_STROBE);

            }
        } else {
            pneumaticsManager.setLedControl(PneumaticsManager.LedControl.AUTO);
        }

//        if (driver.getButtonValue(TRIANGLE) && !previousCurrentBoostButton){
//            if (drivetrainManager.getDriveCurrentLimit() == STALL_LIMIT_DRIVE){
//                drivetrainManager.setDriveCurrentLimit(STALL_LIMIT_DRIVE_CHARGE);
//            } else {
//                drivetrainManager.setDriveCurrentLimit(STALL_LIMIT_DRIVE);
//            }
//        }
//
//        previousCurrentBoostButton = driver.getButtonValue(TRIANGLE);


    }

    @Override
    public void updateBlack() throws HardwareException {
        if (operator.getButtonValue(TRIANGLE) )
        {
            armManager.setCurrentPositionState(ArmManager.PositionState.HIGH);
        }

        //move pivot to middle position
        if (operator.getButtonValue(CIRCLE) )
        {
           armManager.setCurrentPositionState(ArmManager.PositionState.MIDDLE);
        }

        //move pivot to low position
        if (operator.getButtonValue(CROSS) )
        {
           armManager.setCurrentPositionState(ArmManager.PositionState.LOW);
        }

        // move pivot to human player station
        if (operator.getButtonValue(SQUARE) )
        {
            armManager.setCurrentPositionState(ArmManager.PositionState.PLAYER_STATION);
            gripperManager.setControlState(GripperManager.ControlState.INTAKE);
        }

        //Move pivot and telescope to ground intake
        if (operator.getButtonValue(OPTIONS)){
            armManager.setCurrentPositionState(ArmManager.PositionState.GROUND);
        }
//
//        if (armManager.getCurrentPositionState() == ArmManager.PositionState.CONE_SMASH && armManager.isOnTarget()){
//            armManager.setCurrentPositionState(ArmManager.PositionState.SPINDEXER_RETRACTED);
//            if (!gripperManager.getDetectObject()){
//                gripperManager.setControlState(GripperManager.ControlState.DISABLED);
//            }
//        }
//
//        if (operator.getDpadValue(DPAD_W)){
//            armManager.setCurrentPositionState(ArmManager.PositionState.CONE_SMASH);
//            gripperManager.setControlState(GripperManager.ControlState.HOLD);
//        }

        // Bind Dpad south to spindexer position in extended position
        // If spindexer is done we extend telescope and disable spindexer
        // TODO uncomment if needed
//        if (operator.getDpadValue(DPAD_S)|| (intakeManager.getIsOccupied() && !gripperManager.getDetectObject()))
        if (operator.getDpadValue(DPAD_S))
        {
            if (!gripperManager.getDetectObject()) {
                armManager.setCurrentPositionState(ArmManager.PositionState.SPINDEXER_EXTENDED);
            }
        }

        // Bind Dpad north to spindexer position in retracted position
        if (operator.getDpadValue(DPAD_N) || operator.getDpadValue(DPAD_NW) || operator.getDpadValue(DPAD_NE) || operator.getDpadValue(DPAD_W)){
            armManager.setCurrentPositionState(ArmManager.PositionState.SPINDEXER_RETRACTED);
            if (!gripperManager.getDetectObject()){
                gripperManager.setControlState(GripperManager.ControlState.DISABLED);
            } else {
                gripperManager.setControlState(GripperManager.ControlState.HOLD);
            }
        }

        // If arm is extended intake with gripper, then if intaking is done retract arm
        if (armManager.getCurrentPositionState() == ArmManager.PositionState.SPINDEXER_EXTENDED){
            if(gripperManager.getControlState() != GripperManager.ControlState.HOLD){
                gripperManager.setControlState(GripperManager.ControlState.INTAKE);
            } else {
                armManager.setCurrentPositionState(ArmManager.PositionState.SPINDEXER_RETRACTED);
            }
        }

        if (gripperManager.getControlState() == GripperManager.ControlState.HOLD) {
            pneumaticsManager.setLedControl(PneumaticsManager.LedControl.MANUAL);
            pneumaticsManager.setPattern(BlinkinLED.Pattern.SOLID_COLOR_GREEN);
        } else if (drivetrainManager.getDriveCurrentLimit() == STALL_LIMIT_DRIVE) {
            pneumaticsManager.setLedControl(PneumaticsManager.LedControl.AUTO);
        }

        // Gripper manual controls
        if (operator.getAxisValue(TRIGGER_R2) > 0.1 && !armManager.getArmInRobot()) {
            gripperReleased = true;
            // Bind right trigger to gripper outtake
            gripperManager.setControlState(GripperManager.ControlState.OUTTAKE);
        } else if (operator.getAxisValue(TRIGGER_L2) > 0.1){
            // Bind left trigger to gripper intake
            gripperManager.setControlState(GripperManager.ControlState.INTAKE);
        } else if (operator.getButtonValue(BUMPER_R1)&& !armManager.getArmInRobot()) {
            // Bind right bumper to gripper shooting
            gripperManager.setControlState(GripperManager.ControlState.SHOOT);
            gripperReleased = true;
        } else if (gripperManager.getSensorDisconnected()) {
            gripperManager.setControlState(GripperManager.ControlState.HOLD);
        }

        if (gripperReleased && gripperManager.getControlState() == GripperManager.ControlState.DISABLED){
            armManager.setCurrentPositionState(ArmManager.PositionState.SPINDEXER_RETRACTED);
            gripperReleased = false;
        }

        //Auto pickup from spindexer
        if (intakeManager.getIsOccupied() && !gripperManager.getDetectObject() && gamePieceHandler.getGamePiece() == CUBE){
            gripperManager.setControlState(GripperManager.ControlState.INTAKE);
            armManager.setCurrentPositionState(ArmManager.PositionState.SPINDEXER_EXTENDED);
        }

        // Enable or disable spindexer counterclockwise mode
        // TODO do not enable if spindexer is already done
//        if (operator.getRawButtonPressed(SHARE.id)) {
//            if (spindexerManager.getControlState() == SpindexerManager.ControlState.ENABLED) {
//                spindexerManager.setControlState(SpindexerManager.ControlState.DISABLED);
//            } else {
//                spindexerManager.setControlState(SpindexerManager.ControlState.ENABLED);
//            }
//        }
//
//        if (Math.abs(operator.getAxisValue(RIGHTSTICK_X)) > 0.1 || Math.abs(operator.getAxisValue(LEFTSTICK_X)) > 0.1){
//            spindexerManager.setControlState(SpindexerManager.ControlState.MANUAL);
//            spindexerManager.setSpindexerState(SpindexerManager.DirectionState.SPEED);
//            spindexerManager.setSpinSpeed( (operator.getAxisValue(RIGHTSTICK_X) + operator.getAxisValue(LEFTSTICK_X)) / 0.5 );
//        } else if (spindexerManager.getControlState() == SpindexerManager.ControlState.MANUAL){
//            spindexerManager.setControlState(SpindexerManager.ControlState.DISABLED);
//        }




        // Toggle between cube and cone
        if (operator.getRawButtonPressed(BUMPER_L1.id)) {
            if (gamePieceHandler.getGamePiece() == CONE) {
                gamePieceHandler.setGamePiece(CUBE);
            } else {
                gamePieceHandler.setGamePiece(CONE);
            }
        }
        SmartDashboard.putString("Game Piece", gamePieceHandler.getGamePiece().toString());
        SmartDashboard.putBoolean("CONE-CUBE", gamePieceHandler.getGamePiece() == CONE);
        SmartDashboard.putBoolean("x pressed", goWallAngle);

    }

    /**
     * Sets the target velocities of a {@code Drivetrain} equal to the specified input velocities
     *
     * @param forwardVelocity velocity in a forward direction in m/s
     * @param rightVelocity   velocity in a right direction in m/s
     * @param angularVelocity velocity in a counterclockwise direction in rad/s
     */
    private void setDrivetrainSpeed(double forwardVelocity, double rightVelocity, double angularVelocity) {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(forwardVelocity, rightVelocity, angularVelocity);
        if (DriverStation.getAlliance() == DriverStation.Alliance.Red){
            chassisSpeeds = new ChassisSpeeds(-forwardVelocity, -rightVelocity, angularVelocity);
        }
        drivetrainManager.setDesiredSpeeds(chassisSpeeds);
        SmartDashboard.putNumber("DT/slewrated target velocity", driveSpeedSlewRateLimiter.calculate(Math.hypot(forwardVelocity, rightVelocity)));
    }





    /**
     * Filters controllerValue with a deadband
     *
     * @param controllerValue controller value
     * @param deadband        deadband a positive value
     * @return filtered controller value
     */
    private double filterDeadband(double controllerValue, double deadband) {
        if (Math.abs(controllerValue) <= deadband) {
            return 0;
        } else {
            return controllerValue;
        }
    }

    /**
     * Zeroes drivetrain sensors
     *
     * @param button controller button
     */
    private void zeroDrivetrainRotation(boolean button) {
        if (button) {
            if (DriverStation.getAlliance() == DriverStation.Alliance.Red){
                drivetrain.resetRotation(Rotation2d.fromDegrees(180));
            } else {
                drivetrain.resetRotation(new Rotation2d());
            }

//            if (DriverStation.getAlliance() == DriverStation.Alliance.Red){
//                drivetrain.setPigeonYaw(180);
//            }
        }
    }

    /**
     * Make the drivetrain face a certain point on the field
     *
     * @param button controller button
     */
    private void drivetrainLookAtPosition(boolean button) {
        if (button) {
            drivetrain.lookAtPosition(new Translation2d());
        }
    }

    private void autoAlignGrid(){
        Pose2d currentPose = drivetrainManager.getPoseEstimator().getEstimatedPosition();
        Pose2d targetPose = AutoAligner.getTargetPose2d(currentPose, Field.MAX_GRID_DISTANCE, Field.NODE_DEADZONE, armManager.getCurrentPositionState());
        double maxVelocity = AutoAligner.getMaxVelocity(currentPose);

        SmartDashboard.putString("DT/Auto align/Target pose", targetPose.toString());
        SmartDashboard.putNumberArray("DT/Auto align/Target pose arr", new Double[]{targetPose.getX(), targetPose.getY(), targetPose.getRotation().getDegrees()});
        drivetrain.field.getObject("align target").setPose(targetPose);

        Translation2d errorTranslation = targetPose.getTranslation().minus(currentPose.getTranslation());
        Rotation2d errorRotation = targetPose.getRotation().minus(currentPose.getRotation());

        double errorX = errorTranslation.getX();
        double vX = 0;
        if (Math.abs(errorX) > AutoAlign.ALLOWED_ERROR_X){
            vX = errorX * AutoAlign.kP_X;
            vX = MathUtil.clamp(vX,-maxVelocity,maxVelocity);
        }

        double errorY = errorTranslation.getY();
        double vY = 0;
        if (Math.abs(errorY) > AutoAlign.ALLOWED_ERROR_Y){
            vY = errorY * AutoAlign.kP_Y;
            vY = MathUtil.clamp(vY,-maxVelocity,maxVelocity);
        }

        double errorOmega = errorRotation.getRadians();
        double vOmega = errorOmega * ROTATION_KP * MAX_TURN_VELOCITY;

        double targetVelocity = Math.hypot(vX,vY);

        SmartDashboard.putNumber("DT/Auto align/target velocity", targetVelocity);
        SmartDashboard.putNumber("DT/Auto align/error X", errorX);
        SmartDashboard.putNumber("DT/Auto align/error Y", errorY);
        ChassisSpeeds targetSpeeds = new ChassisSpeeds(vX, vY, vOmega);
        drivetrainManager.setDesiredSpeeds(targetSpeeds);
    }

    private void autoAlignKroket(){
//        drivetrainManager.setTargetAligning(true);
//        drivetrainManager.setTargetHeading(Rotation2d.fromDegrees(180));
//        drivetrainManager.setVelocityLimit(2);
        Pose2d currentPose = drivetrainManager.getPoseEstimator().getEstimatedPosition();
        Pose2d targetPose = AutoAligner.getTargetPoseKroket(currentPose,  armManager.getCurrentPositionState());
        double maxVelocity = AutoAligner.getMaxVelocity(currentPose) * 1.5;
        if (drivetrainManager.getUseVision() || humanPlayerAutoAlignPossible){
            humanPlayerAutoAlignPossible = true;
        } else {
            maxVelocity = 0;
        }

        SmartDashboard.putString("DT/Auto align/Target pose", targetPose.toString());
        SmartDashboard.putNumberArray("DT/Auto align/Target pose arr", new Double[]{targetPose.getX(), targetPose.getY(), targetPose.getRotation().getDegrees()});
        drivetrain.field.getObject("align target").setPose(targetPose);

        Translation2d errorTranslation = targetPose.getTranslation().minus(currentPose.getTranslation());
        Rotation2d errorRotation = targetPose.getRotation().minus(currentPose.getRotation());

        double errorX = errorTranslation.getX();
        double vX = 0;
        if (Math.abs(errorX) > AutoAlign.ALLOWED_ERROR_X){
            vX = errorX * AutoAlign.kP_X;
            vX = MathUtil.clamp(vX,-maxVelocity,maxVelocity);
        }

        double errorY = errorTranslation.getY();
        double vY = 0;
        if (Math.abs(errorY) > AutoAlign.ALLOWED_ERROR_Y){
            vY = errorY * AutoAlign.kP_Y;
            vY = MathUtil.clamp(vY,-maxVelocity,maxVelocity);
        }

        double errorOmega = errorRotation.getRadians();
        double vOmega = errorOmega * ROTATION_KP * MAX_TURN_VELOCITY;

        double targetVelocity = Math.hypot(vX,vY);

        SmartDashboard.putNumber("DT/Auto align/target velocity", targetVelocity);
        SmartDashboard.putNumber("DT/Auto align/error X", errorX);
        SmartDashboard.putNumber("DT/Auto align/error Y", errorY);
        ChassisSpeeds targetSpeeds = new ChassisSpeeds(vX, vY, vOmega);
        drivetrainManager.setDesiredSpeeds(targetSpeeds);
    }

    private double curveAxis(double controllerValue) {
        // Exponential
        return controllerValue * Math.exp(Math.abs(controllerValue) - 1);

        // Polynomial
        // return Math.pow(controllerValue, 3);
    }

}





