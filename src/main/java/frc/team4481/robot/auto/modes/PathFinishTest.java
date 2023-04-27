package frc.team4481.robot.auto.modes;

import frc.team4481.lib.auto.actions.WaitAction;
import frc.team4481.lib.auto.mode.AutoModeBase;
import frc.team4481.lib.auto.mode.AutoModeEndedException;
import frc.team4481.robot.Constants;
import frc.team4481.robot.auto.actions.DrivePathAction;
import frc.team4481.robot.auto.actions.IMUBalanceAction;
import frc.team4481.robot.auto.actions.WaitForArmCalibrationAction;
import frc.team4481.robot.auto.selector.AutoMode;
import frc.team4481.robot.auto.selector.Disabled;
import frc.team4481.robot.util.GamePieceHandler;

//@AutoMode(alliance = DriverStation.Alliance.Blue, displayName = "Auto balance test")
@AutoMode(displayName = "[test] path finish")
public class PathFinishTest extends AutoModeBase {
    GamePieceHandler gamePieceHandler;
    String path_1 = "FINISH_PATH_TEST";
    String path_2 = "";
    String path_3 = "";
    String path_4 = "";

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new WaitForArmCalibrationAction());
        //Wait command should be here, otherwise the transition from calibration to the rest won't work
        runAction(new WaitAction(0.1));
        runAction(new DrivePathAction(path_1, false, 0,0));
    }
}
