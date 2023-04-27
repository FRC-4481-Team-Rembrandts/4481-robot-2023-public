package frc.team4481.robot.auto.actions;

import frc.team4481.lib.auto.actions.ParallelAction;
import frc.team4481.robot.subsystems.ArmManager;

public class GrapPieceAction extends ParallelAction {

    /**
     * Creates a new {@code ScorePieceAction} compound action that moves the arm to a position
     * and then releases the game piece.
     */
    public GrapPieceAction(double timeout) {
        super(
                new GripperGrabAction(timeout),
                new SetArmAction(ArmManager.PositionState.SPINDEXER_EXTENDED)

        );
    }
}
