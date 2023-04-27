package frc.team4481.robot.auto.actions;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.team4481.lib.auto.actions.Action;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.robot.subsystems.Drivetrain;
import frc.team4481.robot.subsystems.DrivetrainManager;
import frc.team4481.robot.subsystems.Pneumatics;
import frc.team4481.robot.subsystems.PneumaticsManager;
import frc.team4481.robot.subsystems.output.BlinkinLED;
import pabeles.concurrency.IntProducerNumber;

import static frc.team4481.robot.Constants.Drivetrain.STALL_LIMIT_DRIVE_CHARGE;

public class SetDriveCurrentLimitAction implements Action {
    SubsystemHandler subsystemHandler = SubsystemHandler.getInstance();
    Drivetrain drivetrain;
    DrivetrainManager drivetrainManager;
    Pneumatics pneumatics;
    PneumaticsManager pneumaticsManager;


    public SetDriveCurrentLimitAction(int limit){
        drivetrain = (Drivetrain) subsystemHandler.getSubsystemByClass(Drivetrain.class);
        drivetrainManager = drivetrain.getSubsystemManager();
        pneumatics = (Pneumatics) subsystemHandler.getSubsystemByClass(Pneumatics.class);
        pneumaticsManager = pneumatics.getSubsystemManager();

        drivetrainManager.setDriveCurrentLimit(limit);

        if (drivetrainManager.getDriveCurrentLimit() == STALL_LIMIT_DRIVE_CHARGE){
            pneumaticsManager.setLedControl(PneumaticsManager.LedControl.MANUAL);
            pneumaticsManager.setPattern(BlinkinLED.Pattern.FIRE_MEDIUM);
        } else {
            pneumaticsManager.setLedControl(PneumaticsManager.LedControl.AUTO);
        }


    }



    @Override
    public void start() {
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void done() {
    }
}
