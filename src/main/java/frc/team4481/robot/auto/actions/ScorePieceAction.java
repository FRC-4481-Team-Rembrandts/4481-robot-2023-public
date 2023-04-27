package frc.team4481.robot.auto.actions;

import frc.team4481.lib.auto.actions.SeriesAction;
import frc.team4481.robot.subsystems.ArmManager;

public class ScorePieceAction extends SeriesAction {

    /**
     * Creates a new {@code ScorePieceAction} compound action that moves the arm to a position
     * and then releases the game piece.
     *
     * @param targetPosition
     */
    public ScorePieceAction(ArmManager.PositionState targetPosition) {
        super(
                new SetArmAction(targetPosition),
                new GripperReleaseAction()
        );
    }
}
