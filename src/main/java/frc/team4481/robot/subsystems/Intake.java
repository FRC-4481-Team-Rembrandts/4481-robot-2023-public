package frc.team4481.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4481.lib.hardware.LazyCANSparkMax;
import frc.team4481.lib.subsystems.SubsystemBase;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.lib.util.CountingDelay;
import frc.team4481.robot.util.GamePieceHandler;

import static frc.team4481.robot.util.GamePieceHandler.GamePiece.*;
import static frc.team4481.robot.Constants.Intake.*;
import static frc.team4481.robot.Constants.HardwareMap.*;
import static frc.team4481.robot.Constants.HardwareMap.Comp.*;

public class Intake extends SubsystemBase<IntakeManager> {
    private final SubsystemHandler subsystemHandler = SubsystemHandler.getInstance();
    private final GamePieceHandler gamePieceHandler = GamePieceHandler.getInstance();

    private LazyCANSparkMax motor;
    private RelativeEncoder encoder;
    private Solenoid solenoid;
    private boolean intakePos;
    private double speed;
    private AnalogInput occupancySensor;
    private CountingDelay intakeRollingDelay;
    private CountingDelay sensorDelay;

    public Intake(){
        name = "Intake";
        subsystemManager = new IntakeManager();

        motor = new LazyCANSparkMax(INTAKE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        motor.setInverted(INVERT_MOTOR);
        encoder = motor.getEncoder();
        solenoid = new Solenoid(PneumaticsModuleType.REVPH, INTAKE_SOLENOID);
        intakeRollingDelay = new CountingDelay();
        sensorDelay = new CountingDelay();

        //dexer
        occupancySensor = new AnalogInput(OCCUPANCY_SENSOR_ANALOG_CHANNEL);
    }

    @Override
    public void onStart(double timestamp) {
        subsystemManager.setControlState(IntakeManager.ControlState.DISABLED);
        zeroSensors();
    }

    @Override
    public void readPeriodicInputs() {
        if (occupancySensor.getValue() >= OCCUPANCY_SENSOR_THRESHOLD){
            sensorDelay.reset();
        }
        subsystemManager.setIsOccupied(sensorDelay.delay(OCCUPANCY_SENSOR_DELAY));

    }

    @Override
    public void onLoop(double timestamp) {
        IntakeManager.ControlState state = subsystemManager.getControlState();
        if (!intakeRollingDelay.delay(PASSIVE_ROLL_TIME) && state == IntakeManager.ControlState.DISABLED) {
            state = IntakeManager.ControlState.ROLLING;
        }

        switch (state) {
            case DISABLED:
                intakePos = !INTAKE_OPEN;
                speed = 0.0;
                break;
            case INTAKE:
                intakePos = INTAKE_OPEN;
                if (gamePieceHandler.getGamePiece() == CUBE) {
                    speed = CUBE_SPEED;
                } else {
                    speed = CONE_SPEED;
                }
                intakeRollingDelay.reset();
                intakeRollingDelay.delay(0);//start timer
                break;
            case REVERSE:
                intakePos = INTAKE_OPEN;
                if (gamePieceHandler.getGamePiece() == CUBE) {
                    speed = -CUBE_SPEED;
                } else {
                    speed = -CONE_SPEED;
                }
                break;
            case ROLLING:
                intakePos = !INTAKE_OPEN;
                speed = CONE_SPEED/4;
                break;
            case TUNING:
                intakePos = INTAKE_OPEN;
                speed = 0.0;
                break;
        }
        SmartDashboard.putString("intake/control state", state.toString());
    }
    @Override
    public void writePeriodicOutputs() {
        setIntakePosition(intakePos);
        setRollerSpeed(speed);
    }

    @Override
    public void onStop(double timestamp) {
        terminate();
    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public void terminate() {
        subsystemManager.setControlState(IntakeManager.ControlState.DISABLED);
    }

    @Override
    public void outputData() {
        SmartDashboard.putNumber("intake/motor percent", motor.getAppliedOutput());
        SmartDashboard.putNumber("intake/Motor RPM", encoder.getVelocity());
        SmartDashboard.putBoolean("intake/Dexer occupy", subsystemManager.getIsOccupied());
        SmartDashboard.putNumber("intake/occupy sensor raw", occupancySensor.getValue());
    }

    /**
     * It sets the rollerspeed.
     * @param rollerSpeed a speed of -1 to 1
     */
    private void setRollerSpeed(double rollerSpeed) {
        motor.set(rollerSpeed);
    }

    /**
     * It sets the intake Position.
     * @param onOrOff On is unfold, off is fold in
     */
    private void setIntakePosition(boolean onOrOff) {
        solenoid.set(onOrOff);
    }

    /**
     * Chassis speed to absolute speed
     * @param chassisSpeeds
     *
     * @return absolute chassis speed in meter per second
     */
    private double chassisSpeedToSpeed(ChassisSpeeds chassisSpeeds) {
        double vx = chassisSpeeds.vxMetersPerSecond;
        double vy = chassisSpeeds.vyMetersPerSecond;
        return Math.hypot(vx, vy);
    }
}