package frc.team4481.robot.HIDlayout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.team4481.lib.controller.ControlDevice;
import frc.team4481.lib.hid.HIDLayout;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.lib.throwable.HardwareException;
import frc.team4481.robot.path.AutoAligner;
import frc.team4481.robot.subsystems.Drivetrain;
import frc.team4481.robot.subsystems.DrivetrainManager;

import static frc.team4481.lib.controller.IPS4HID.Axis.*;
import static frc.team4481.lib.controller.IPS4HID.Button.*;
import static frc.team4481.robot.Constants.*;

public class SwerveLayout extends HIDLayout {
    private final SubsystemHandler mSubsystemHandler = SubsystemHandler.getInstance();
    private Drivetrain drivetrain;
    private DrivetrainManager drivetrainManager;

    public SwerveLayout(ControlDevice driver, ControlDevice operator) {
        super(driver, operator);
    }

    @Override
    public void getSubsystemManagers() {
        drivetrain = (Drivetrain) mSubsystemHandler.getSubsystemByClass(Drivetrain.class);
        drivetrainManager = drivetrain.getSubsystemManager();
    }

    @Override
    public void updateOrange() throws HardwareException {
        if(driver.getButtonValue(TRIANGLE)) {
            Pose2d currentPose = drivetrainManager.getPoseEstimator().getEstimatedPosition();
            Pose2d targetPose = AutoAligner.getTargetPose2d(currentPose, 10, 0.05, null);

            ChassisSpeeds targetSpeeds = new ChassisSpeeds();

            if (currentPose.getTranslation().getDistance(targetPose.getTranslation()) > 0.05) {
                targetSpeeds =
                        drivetrain.aPPController.getChassisSpeedsForTargetDelta(currentPose, targetPose, 1);
            }

            drivetrainManager.setDesiredSpeeds(targetSpeeds);
        } else {
            setDrivetrainSpeed(
                    filterDeadband(-driver.getAxisValue(LEFTSTICK_Y), ORANGE_LEFTSTICK_DEADBAND),
                    filterDeadband(-driver.getAxisValue(LEFTSTICK_X), ORANGE_LEFTSTICK_DEADBAND),
                    filterDeadband(-driver.getAxisValue(RIGHTSTICK_X), ORANGE_RIGHTSTICK_DEADBAND)
            );
        }

        zeroDrivetrainSensors(driver.getButtonValue(CROSS));
        drivetrainLookAtPosition(driver.getButtonValue(CIRCLE));
        boostDrivetrain(driver.getButtonValue(BUMPER_R1));
    }

    @Override
    public void updateBlack() throws HardwareException {

    }

    /**
     * Sets the target velocities of a {@code Drivetrain} equal to the specified input velocities
     *
     * @param forwardVelocity velocity in a forward direction in m/s
     * @param rightVelocity   velocity in a right direction in m/s
     * @param angularVelocity velocity in a counterclockwise direction in rad/s
     */
    private void setDrivetrainSpeed(double forwardVelocity, double rightVelocity, double angularVelocity) {
        double forwardChassisSpeed;
        double rightChassisSpeed;
        double angularChassisSpeed = angularVelocity * MAX_TURN_VELOCITY;

        if (/*drivetrainManager.getBoost() */ false) {
            forwardChassisSpeed = forwardVelocity * MAX_VELOCITY;
            rightChassisSpeed = rightVelocity * MAX_VELOCITY;
        } else {
            forwardChassisSpeed = forwardVelocity * MAX_VELOCITY;
            rightChassisSpeed = rightVelocity * MAX_VELOCITY;
        }

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(forwardChassisSpeed, rightChassisSpeed, angularChassisSpeed);
        drivetrainManager.setDesiredSpeeds(chassisSpeeds);
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
    private void zeroDrivetrainSensors(boolean button) {
        if (button) {
            drivetrain.zeroSensors();
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

    /**
     * When button is pressed, increase max velocity
     * of the drivetrain
     *
     * @param button controller button
     */
    private void boostDrivetrain(boolean button) {
//        drivetrainManager.setBoost(button);
    }
}
