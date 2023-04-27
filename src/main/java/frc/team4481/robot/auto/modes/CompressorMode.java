package frc.team4481.robot.auto.modes;

import frc.team4481.lib.auto.actions.WaitAction;
import frc.team4481.lib.auto.mode.AutoModeBase;
import frc.team4481.lib.auto.mode.AutoModeEndedException;
import frc.team4481.robot.auto.actions.WaitForArmCalibrationAction;
import frc.team4481.robot.auto.selector.AutoMode;

/**
 * Default AutoMode that does nothing but calibrate the arm.
 */
@AutoMode(displayName = "[Queue] PompenPompenPompen")
public class CompressorMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new WaitForArmCalibrationAction());
        runAction(new WaitAction(600));
    }
}
