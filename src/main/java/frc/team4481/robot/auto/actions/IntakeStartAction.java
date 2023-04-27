package frc.team4481.robot.auto.actions;
import frc.team4481.lib.auto.actions.Action;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.robot.subsystems.Gripper;
import frc.team4481.robot.subsystems.GripperManager;
import frc.team4481.robot.subsystems.Intake;
import frc.team4481.robot.subsystems.IntakeManager;

/**
 * Action to drop a cone
 */
public class IntakeStartAction implements Action {
    SubsystemHandler subsystemHandler = SubsystemHandler.getInstance();
    IntakeManager intakeManager;
    Intake intake;
    Gripper gripper;
    GripperManager gripperManager;
    public IntakeStartAction(){
        intake = (Intake) subsystemHandler.getSubsystemByClass(Intake.class);
        intakeManager = intake.getSubsystemManager();
        gripper = (Gripper) subsystemHandler.getSubsystemByClass(Gripper.class);
        gripperManager = gripper.getSubsystemManager();
    }
    public void start() {
        if (!intakeManager.getIsOccupied() || gripperManager.getDetectObject()){
            intakeManager.setControlState(IntakeManager.ControlState.INTAKE);
        }
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void done() {

    }
}
