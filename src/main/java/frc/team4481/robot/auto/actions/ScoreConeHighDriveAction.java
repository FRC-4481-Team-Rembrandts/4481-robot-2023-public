package frc.team4481.robot.auto.actions;

import frc.team4481.lib.auto.actions.ParallelAction;
import frc.team4481.lib.auto.actions.SeriesAction;
import frc.team4481.lib.auto.actions.WaitAction;
import frc.team4481.robot.subsystems.ArmManager;
import frc.team4481.robot.util.GamePieceHandler;

public class ScoreConeHighDriveAction extends SeriesAction {
    /**
     * Action to move the arm to high and release a cone,
     * the arm should still be moved to SPINDEXER_RETRACTED afterwards
     */
    public ScoreConeHighDriveAction(String pathName, boolean reversed, double maxVelocity, double maxAcceleration){
        super(
                new ParallelAction(
                        new SeriesAction(
                                new ScoreConeHighAction(),
                                new WaitAction(0.09),
                                new SetArmAction(ArmManager.PositionState.SPINDEXER_RETRACTED)
                        ),

                        new SeriesAction(
                                new WaitAction(1.2),
                                new DrivePathAction(pathName, reversed,maxVelocity,maxAcceleration)
                        )
                )
        );
    }
}
