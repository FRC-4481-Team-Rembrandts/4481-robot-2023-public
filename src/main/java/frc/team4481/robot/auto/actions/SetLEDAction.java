package frc.team4481.robot.auto.actions;
import frc.team4481.lib.auto.actions.Action;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.robot.subsystems.*;
import frc.team4481.robot.subsystems.output.BlinkinLED;

/**
 * Action to drop a cone
 */
public class SetLEDAction implements Action {
    SubsystemHandler subsystemHandler = SubsystemHandler.getInstance();
    PneumaticsManager pneumaticsManager;
    Pneumatics pneumatics;

    PneumaticsManager.LedControl ledMode;
    BlinkinLED.Pattern ledPattern;

    public SetLEDAction(){
        pneumatics = (Pneumatics) subsystemHandler.getSubsystemByClass(Pneumatics.class);
        pneumaticsManager = pneumatics.getSubsystemManager();

        this.ledMode = PneumaticsManager.LedControl.AUTO;
        this.ledPattern = BlinkinLED.Pattern.SOLID_COLOR_BLACK;
    }

    public SetLEDAction(BlinkinLED.Pattern ledPattern){
        pneumatics = (Pneumatics) subsystemHandler.getSubsystemByClass(Pneumatics.class);
        pneumaticsManager = pneumatics.getSubsystemManager();

        this.ledMode = PneumaticsManager.LedControl.MANUAL;
        this.ledPattern = ledPattern;
    }

    public void start() {
        pneumaticsManager.setLedControl(ledMode);
        pneumaticsManager.setPattern(ledPattern);
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
