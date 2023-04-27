package frc.team4481.robot.auto.actions;
import frc.team4481.lib.auto.actions.Action;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.lib.util.CountingDelay;
import frc.team4481.robot.subsystems.*;
import frc.team4481.robot.subsystems.output.BlinkinLED;

/**
 * Action to grab a game piece from the dexer
 */
public class GripperGrabAction implements Action {
    SubsystemHandler subsystemHandler = SubsystemHandler.getInstance();
    GripperManager gripperManager;
    Gripper gripper;
    Pneumatics pneumatics;
    PneumaticsManager pneumaticsManager;
    CountingDelay stopDelay;
    double timeout;

    public GripperGrabAction(double timeout){
        gripper = (Gripper) subsystemHandler.getSubsystemByClass(Gripper.class);
        gripperManager = gripper.getSubsystemManager();
        pneumatics = (Pneumatics) subsystemHandler.getSubsystemByClass(Pneumatics.class);
        pneumaticsManager = pneumatics.getSubsystemManager();
        stopDelay = new CountingDelay();
        this.timeout = timeout;
    }
    public void start() {
        gripperManager.setControlState(GripperManager.ControlState.INTAKE);
        stopDelay.reset();
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return gripperManager.getControlState() == GripperManager.ControlState.HOLD
                || stopDelay.delay(timeout);
    }

    @Override
    public void done() {
        pneumaticsManager.setLedControl(PneumaticsManager.LedControl.MANUAL);
        pneumaticsManager.setPattern(BlinkinLED.Pattern.SOLID_COLOR_GREEN);
    }
}
