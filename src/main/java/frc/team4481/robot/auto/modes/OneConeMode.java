package frc.team4481.robot.auto.modes;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.team4481.lib.auto.actions.ParallelAction;
import frc.team4481.lib.auto.actions.SeriesAction;
import frc.team4481.lib.auto.actions.WaitAction;
import frc.team4481.lib.auto.mode.AutoModeBase;
import frc.team4481.lib.auto.mode.AutoModeEndedException;
import frc.team4481.robot.auto.actions.*;
import frc.team4481.robot.auto.selector.AutoMode;
import frc.team4481.robot.subsystems.ArmManager;
import frc.team4481.robot.util.GamePieceHandler;

/**
 * Scores initial cone
 * collects cube closest to cable duct
 * scores high closest to cable duct
 * collects next cube
 * scores mid closest to cable duct
 */
@AutoMode(displayName = "[All] 1 Cone Score")
public class OneConeMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new ScoreConeHighAction());
        runAction(new SetArmAction(ArmManager.PositionState.SPINDEXER_RETRACTED));
    }
}
