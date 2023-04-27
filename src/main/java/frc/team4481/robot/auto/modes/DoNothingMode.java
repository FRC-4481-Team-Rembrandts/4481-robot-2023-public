package frc.team4481.robot.auto.modes;

import frc.team4481.lib.auto.mode.AutoModeBase;
import frc.team4481.lib.auto.mode.AutoModeEndedException;
import frc.team4481.robot.auto.actions.WaitForArmCalibrationAction;

/**
 * Default AutoMode that does nothing but calibrate the arm.
 */
public class DoNothingMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new WaitForArmCalibrationAction());
    }
}
