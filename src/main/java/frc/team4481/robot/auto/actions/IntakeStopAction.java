package frc.team4481.robot.auto.actions;
import frc.team4481.lib.auto.actions.Action;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.robot.subsystems.Intake;
import frc.team4481.robot.subsystems.IntakeManager;

/**
 * Action to drop a cone
 */
public class IntakeStopAction implements Action {
    SubsystemHandler subsystemHandler = SubsystemHandler.getInstance();
    IntakeManager intakeManager;
    Intake intake;

    public IntakeStopAction(){
        intake = (Intake) subsystemHandler.getSubsystemByClass(Intake.class);
        intakeManager = intake.getSubsystemManager();
    }
    public void start() {
        intakeManager.setControlState(IntakeManager.ControlState.DISABLED);
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
