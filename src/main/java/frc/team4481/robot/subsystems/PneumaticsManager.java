package frc.team4481.robot.subsystems;

import frc.team4481.lib.subsystems.SubsystemManagerBase;
import frc.team4481.robot.subsystems.output.BlinkinLED;

public class PneumaticsManager extends SubsystemManagerBase {
    private double pneumaticsPressure = -1;
    private boolean compressorState = false;
    private BlinkinLED.Pattern pattern = BlinkinLED.Pattern.C1_BREATH_SLOW;
    private LedControl ledControl = LedControl.AUTO;

    public enum LedControl {
        AUTO,
        MANUAL
    }

    public double getPneumaticsPressure() {
        return pneumaticsPressure;
    }

    public void setPneumaticsPressure(double pneumaticsPressure) {
        this.pneumaticsPressure = pneumaticsPressure;
    }

    public boolean getCompressorState() {
        return compressorState;
    }

    public void setCompressorState(boolean compressorState) {
        this.compressorState = compressorState;
    }

    public BlinkinLED.Pattern getPattern() {
        return pattern;
    }

    public void setPattern(BlinkinLED.Pattern pattern) {
        this.pattern = pattern;
    }

    public LedControl getLedControl() {
        return ledControl;
    }

    public void setLedControl(LedControl ledControl) {
        this.ledControl = ledControl;
    }
}
