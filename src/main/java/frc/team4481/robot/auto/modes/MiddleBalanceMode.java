package frc.team4481.robot.auto.modes;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.team4481.lib.auto.actions.WaitAction;
import frc.team4481.lib.auto.mode.AutoModeBase;
import frc.team4481.lib.auto.mode.AutoModeEndedException;
import frc.team4481.robot.Constants;
import frc.team4481.robot.auto.actions.DrivePathAction;
import frc.team4481.robot.auto.actions.IMUBalanceAction;
import frc.team4481.robot.auto.actions.ScoreConeHighDriveAction;
import frc.team4481.robot.auto.selector.AutoMode;

/**
 * Scores initial cone
 * collects cube closest to cable duct
 * scores high closest to cable duct
 * collects next cube
 * scores mid closest to cable duct
 */
@AutoMode(displayName = "[Middle] 1 Cone Balance")
public class MiddleBalanceMode extends AutoModeBase {
    double startTime;

    String path1 = "MIDDLE_MOBILITY";
    String path2 = "MIDDLE_BALANCE";

    @Override
    protected void routine() throws AutoModeEndedException {
        startTime = Timer.getFPGATimestamp();

        if (DriverStation.getAlliance() == DriverStation.Alliance.Red){
            path1 = "MIDDLE_MOBILITY_FLIPPED";
            path2 = "MIDDLE_BALANCE_FLIPPED";
        }
        runAction(new ScoreConeHighDriveAction(path1,false, 1,Constants.MAX_ACCELERATION));
        runAction(new WaitAction(0.2));
        runAction(new DrivePathAction(path2, false, 1, Constants.MAX_ACCELERATION));
        runAction(new IMUBalanceAction(Constants.AUTO_BALANCE_VELOCITY, Constants.AUTO_BALANCE_MAX_STOP_ERROR, Constants.AUTO_BALANCE_KP, startTime));
        runAction(new WaitAction(0.1));

    }
}
