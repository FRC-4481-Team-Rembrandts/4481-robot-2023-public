package frc.team4481.robot.auto.actions;
import frc.team4481.lib.auto.actions.Action;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.robot.subsystems.Gripper;
import frc.team4481.robot.subsystems.GripperManager;
import frc.team4481.robot.subsystems.Pneumatics;
import frc.team4481.robot.subsystems.PneumaticsManager;

/**
 * Action to drop a cone
 */
public class GripperReleaseAction implements Action {
    SubsystemHandler subsystemHandler = SubsystemHandler.getInstance();
    GripperManager gripperManager;
    Gripper gripper;
    Pneumatics pneumatics;
    PneumaticsManager pneumaticsManager;

    public GripperReleaseAction(){
        gripper = (Gripper) subsystemHandler.getSubsystemByClass(Gripper.class);
        gripperManager = gripper.getSubsystemManager();
        pneumatics = (Pneumatics) subsystemHandler.getSubsystemByClass(Pneumatics.class);
        pneumaticsManager = pneumatics.getSubsystemManager();
    }
    public void start() {
        gripperManager.setControlState(GripperManager.ControlState.OUTTAKE);
        pneumaticsManager.setLedControl(PneumaticsManager.LedControl.AUTO);
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
