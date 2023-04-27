package frc.team4481.robot.auto.actions;
import frc.team4481.lib.auto.actions.Action;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.robot.subsystems.Intake;
import frc.team4481.robot.subsystems.IntakeManager;
//import frc.team4481.robot.subsystems.Spindexer;
//import frc.team4481.robot.subsystems.SpindexerManager;

/**
 * Action to drop a cone
 */
public class SetIntakeAction implements Action {
    SubsystemHandler subsystemHandler = SubsystemHandler.getInstance();
    IntakeManager intakeManager;
    Intake intake;
//    SpindexerManager spindexerManager;
//    Spindexer spindexer;
    IntakeManager.ControlState controlState;

    public SetIntakeAction(IntakeManager.ControlState controlState){
        intake = (Intake) subsystemHandler.getSubsystemByClass(Intake.class);
        intakeManager = intake.getSubsystemManager();
//        spindexer = (Spindexer) subsystemHandler.getSubsystemByClass(Spindexer.class);
//        spindexerManager = spindexer.getSubsystemManager();
        this.controlState = controlState;
    }
    public void start() {
        intakeManager.setControlState(controlState);
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
