package frc.team4481.robot.auto.actions;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4481.lib.auto.actions.Action;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.lib.util.CountingDelay;
import frc.team4481.robot.subsystems.Drivetrain;
import frc.team4481.robot.subsystems.DrivetrainManager;

/**
 * Action that balances a robot until the CHARGE STATION is ENGAGED provided that
 * it initially started DOCKED.
 */
public class IMUBalanceAction implements Action {
    SubsystemHandler subsystemHandler = SubsystemHandler.getInstance();
    Drivetrain drivetrain;
    DrivetrainManager drivetrainManager;
    Pigeon2 pigeon;

    double maxVelocity;
    double maxStopError;
    double kP;
    CountingDelay stopDelay;
    double timeout;
    double autonomousStartTime;

    /**
     * Creates a new IMUBalanceAction that balances a robot until the CHARGE STATION is ENGAGED provided that
     * it initially started DOCKED.
     */
    public IMUBalanceAction(double maxVelocity, double maxStopError, double kP, double autonomousStartTime){
        drivetrain = (Drivetrain) subsystemHandler.getSubsystemByClass(Drivetrain.class);
        drivetrainManager = drivetrain.getSubsystemManager();
        pigeon = drivetrain.pigeon;

        this.maxVelocity = maxVelocity;
        this.maxStopError = maxStopError;
        this.kP = kP;
        this.autonomousStartTime = autonomousStartTime;
    }


    /**
     * Creates a new IMUBalanceAction that balances a robot until the CHARGE STATION is ENGAGED provided that
     * it initially started DOCKED.
     */
    public IMUBalanceAction(double maxVelocity, double maxStopError, double kP){
            drivetrain = (Drivetrain) subsystemHandler.getSubsystemByClass(Drivetrain.class);
            drivetrainManager = drivetrain.getSubsystemManager();
            pigeon = drivetrain.pigeon;

            this.maxVelocity = maxVelocity;
            this.maxStopError = maxStopError;
            this.kP = kP;
            this.autonomousStartTime = 10000;
    }

    @Override
    public void start() {
        drivetrainManager.setControlState(DrivetrainManager.controlState.ENABLED);
        timeout = 14.85 - (Timer.getFPGATimestamp() - autonomousStartTime);
        stopDelay = new CountingDelay();
        stopDelay.reset();
    }

    @Override
    public void update() {
        drivetrainManager.setControlState(DrivetrainManager.controlState.ENABLED);

        Rotation3d rotationRobotSpace = new Rotation3d(Math.toRadians(pigeon.getRoll()), Math.toRadians(pigeon.getPitch()), Math.toRadians(pigeon.getYaw()));
        Rotation3d rotationFieldSpace = rotationRobotSpace.rotateBy(new Rotation3d(0,0,Math.toRadians(-pigeon.getYaw())));

        double allianceMultiplier = 1;

        if(DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
            allianceMultiplier = -1;
        }

        double correctedPitch = Math.toDegrees(rotationFieldSpace.getY());

        double xVelocity = 0;
        if (Math.abs(correctedPitch) > maxStopError) {

            xVelocity = allianceMultiplier * -MathUtil.clamp(correctedPitch * maxVelocity * kP, -maxVelocity, maxVelocity);
            drivetrainManager.setLocked(false);
        } else {
            drivetrainManager.setLocked(true);
        }
        DataLogManager.log("Balance velocity " + xVelocity);
        DataLogManager.log("Corrected pitch " + correctedPitch);
        DataLogManager.log("Auto balance timout delay: " + timeout);
        drivetrainManager.setDesiredSpeeds(new ChassisSpeeds(xVelocity,0,0));
    }

    @Override
    public boolean isFinished() {
        return stopDelay.delay(timeout);
    }

    @Override
    public void done() {
        drivetrainManager.setLocked(true);
    }
}
