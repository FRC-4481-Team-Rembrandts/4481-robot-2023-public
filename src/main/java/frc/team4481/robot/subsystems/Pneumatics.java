package frc.team4481.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4481.lib.subsystems.SubsystemBase;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.robot.subsystems.output.BlinkinLED;
import frc.team4481.robot.util.GamePieceHandler;

import static frc.team4481.robot.Constants.Pneumatics.*;
import static frc.team4481.robot.Constants.HardwareMap.*;

public class Pneumatics extends SubsystemBase<PneumaticsManager> {
    private final SubsystemHandler subsystemHandler = SubsystemHandler.getInstance();

    private Relay compressorRelay = new Relay(COMPRESSOR_RELAY_CHANNEL, Relay.Direction.kReverse);
    private AnalogInput pressureSensor = new AnalogInput(PRESSURE_ANALOG_CHANNEL);

    // LED controller woont hier :(
    private final BlinkinLED ledController = new BlinkinLED(LED_CONTROLLER);

    public Pneumatics(){
        name = "Pneumatics";
        subsystemManager = new PneumaticsManager();
    }

    @Override
    public void onStart(double timestamp) {
        subsystemManager.setCompressorState(false);
    }

    @Override
    public void onLoop(double timestamp) {
        double pressureVoltage = pressureSensor.getVoltage();
        double pressurePsi = 250 * (pressureVoltage / 5) - 25;
        subsystemManager.setPneumaticsPressure(pressurePsi);


        boolean enableCompressor = subsystemManager.getCompressorState();

        double pressure_threshold_enable = PRESSURE_THRESHOLD_ENABLE_MATCH;
        double pressure_threshold_disable = PRESSURE_THRESHOLD_DISABLE_MATCH;

        if (DriverStation.isAutonomous()){
            pressure_threshold_enable = PRESSURE_THRESHOLD_ENABLE_QUEUE;
            pressure_threshold_disable = PRESSURE_THRESHOLD_DISABLE_QUEUE;
        }

        if ( enableCompressor && (pressurePsi > pressure_threshold_disable) ) {
            subsystemManager.setCompressorState(false);
        }

        if ( !enableCompressor && (pressurePsi < pressure_threshold_enable) ){
            subsystemManager.setCompressorState(true);
        }

        // Helemaal stroef of nie
        setLED();
    }

    @Override
    public void onStop(double timestamp) {

    }
    @Override
    public void readPeriodicInputs() {

    }

    @Override
    public void writePeriodicOutputs() {
        setCompressorRelayState(subsystemManager.getCompressorState());
    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public void terminate() {
        subsystemManager.setCompressorState(false);
        setCompressorRelayState(subsystemManager.getCompressorState());
    }

    @Override
    public void outputData() {
        SmartDashboard.putNumber("Pressure/sensor value", pressureSensor.getValue());
        SmartDashboard.putNumber("Pressure/sensor voltage", pressureSensor.getVoltage());
        SmartDashboard.putString("Pressure/relay state", compressorRelay.get().getPrettyValue());
        SmartDashboard.putNumber("Pressure/psi", subsystemManager.getPneumaticsPressure());
        SmartDashboard.putBoolean("Pressure/Enable compressor", subsystemManager.getCompressorState());

        SmartDashboard.putString("LED/led control", subsystemManager.getLedControl().toString());
        SmartDashboard.putString("LED/led pattern", subsystemManager.getPattern().toString());
    }

    private void setCompressorRelayState(boolean state) {
        if (state) {
            compressorRelay.set(Relay.Value.kReverse);
        } else {
            compressorRelay.set(Relay.Value.kOff);
        }
    }

    /**
     * The LED controller lives here by lack of better placement
     */
    private void setLED(){
        if (subsystemManager.getLedControl() == PneumaticsManager.LedControl.AUTO) {
            if (GamePieceHandler.getInstance().getGamePiece() == GamePieceHandler.GamePiece.CUBE) {
                ledController.setPattern(BlinkinLED.Pattern.SOLID_COLOR_VIOLET);
            } else {
                ledController.setPattern(BlinkinLED.Pattern.SOLID_COLOR_GOLD);
            }
        } else {
            ledController.setPattern(subsystemManager.getPattern());
        }

    }
}

