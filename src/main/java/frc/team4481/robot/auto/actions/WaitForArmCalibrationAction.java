package frc.team4481.robot.auto.actions;

import frc.team4481.lib.auto.actions.Action;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.robot.subsystems.Arm;
import frc.team4481.robot.subsystems.ArmManager;

public class WaitForArmCalibrationAction implements Action {
    SubsystemHandler subsystemHandler = SubsystemHandler.getInstance();
    Arm arm;
    ArmManager armManager;
    ArmManager.PositionState targetPosition;

    public WaitForArmCalibrationAction() {
        arm = (Arm) subsystemHandler.getSubsystemByClass(Arm.class);
        armManager = arm.getSubsystemManager();
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return armManager.getCurrentControlState() == ArmManager.ControlState.AUTOMATIC;
    }

    @Override
    public void done() {

    }
}
