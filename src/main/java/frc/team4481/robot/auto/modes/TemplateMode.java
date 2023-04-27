package frc.team4481.robot.auto.modes;

import frc.team4481.lib.auto.actions.WaitAction;
import frc.team4481.lib.auto.mode.AutoModeBase;
import frc.team4481.lib.auto.mode.AutoModeEndedException;
import frc.team4481.robot.auto.actions.WaitForArmCalibrationAction;
import frc.team4481.robot.util.GamePieceHandler;

public class TemplateMode extends AutoModeBase {
    GamePieceHandler gamePieceHandler;
    String path_1 = "";
    String path_2 = "";
    String path_3 = "";
    String path_4 = "";

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new WaitForArmCalibrationAction());
        //Wait command should be here, otherwise the transition from calibration to the rest won't work
        runAction(new WaitAction(0.1));


    }
}
