package frc.team4481.robot.auto.actions;
import frc.team4481.lib.auto.actions.ParallelAction;
import frc.team4481.lib.auto.actions.SeriesAction;
import frc.team4481.lib.auto.actions.WaitAction;
import frc.team4481.robot.subsystems.*;
import frc.team4481.robot.util.GamePieceHandler;

public class ScoreConeHighAction extends SeriesAction {
    /**
     * Action to move the arm to high and release a cone,
     * the arm should still be moved to SPINDEXER_RETRACTED afterwards
     */
    public ScoreConeHighAction(){
        super(
                new SelectGamePieceAction(GamePieceHandler.GamePiece.CONE),
                new WaitAction(0.04),
                new ParallelAction(
                        new SetArmAction(ArmManager.PositionState.HIGH),
                        new SeriesAction(
                                new WaitAction(0.96),
                                new GripperReleaseAction()
                        )
                )
        );
    }
}
