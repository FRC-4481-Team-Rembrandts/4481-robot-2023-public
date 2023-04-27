package frc.team4481.robot.auto.actions;

import edu.wpi.first.wpilibj.DataLogManager;
import frc.team4481.lib.auto.actions.Action;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.robot.subsystems.Arm;
import frc.team4481.robot.subsystems.ArmManager;

public class SetArmAction implements Action {
    SubsystemHandler subsystemHandler = SubsystemHandler.getInstance();
    Arm arm;
    ArmManager armManager;
    ArmManager.PositionState targetPosition;

    public SetArmAction(ArmManager.PositionState targetPosition) {
        arm = (Arm) subsystemHandler.getSubsystemByClass(Arm.class);
        armManager = arm.getSubsystemManager();

        this.targetPosition = targetPosition;
    }

    @Override
    public void start() {
        armManager.setUpdated(false);
        armManager.setOnTarget(false);
        armManager.setCurrentPositionState(targetPosition);
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return armManager.isOnTarget();
    }

    @Override
    public void done() {
        DataLogManager.log("Set arm action done");
    }
}
