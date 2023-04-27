package frc.team4481.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4481.lib.hardware.LazyCANSparkMax;
import frc.team4481.lib.subsystems.SubsystemBase;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.lib.util.CountingDelay;
import frc.team4481.robot.util.GamePieceHandler;

import static frc.team4481.robot.Constants.HardwareMap.*;
import static frc.team4481.robot.Constants.HardwareMap.Comp.*;
import static frc.team4481.robot.Constants.Gripper.*;
import static frc.team4481.robot.Constants.Gripper.Comp.*;
import static frc.team4481.robot.subsystems.GripperManager.ControlState.*;

public class Gripper extends SubsystemBase<GripperManager> {
    private final SubsystemHandler subsystemHandler = SubsystemHandler.getInstance();
    private final GamePieceHandler gamePieceHandler = GamePieceHandler.getInstance();

    private final LazyCANSparkMax gripperMotor;
    private SparkMaxPIDController gripperController;
    private RelativeEncoder gripperEncoder;
    private double holdPosition;
    private final Solenoid gripperPiston;
    private CountingDelay autoGripperDelay;
    private CountingDelay sensorDefectCount;
    private GripperManager.ControlState prevControlState = DISABLED;
    private GamePieceHandler.GamePiece prevGamePiece = GamePieceHandler.GamePiece.CONE;
    private boolean enterHold = true;

    public Gripper(){
        name = "Gripper";
        subsystemManager = new GripperManager();
        gripperMotor = new LazyCANSparkMax(GRIPPER_MOTOR, CANSparkMax.MotorType.kBrushless);
        gripperMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        gripperMotor.setSmartCurrentLimit(GRIPPER_CUBE_CURRENT_LIMIT);

        gripperEncoder = gripperMotor.getEncoder();

        gripperController = gripperMotor.getPIDController();
        gripperController.setFeedbackDevice(gripperEncoder);
        gripperController.setP(1, 0);
        gripperController.setI(0,0);
        gripperController.setD(0,0);

        gripperMotor.burnFlash();

        gripperPiston = new Solenoid(PneumaticsModuleType.REVPH,GRIPPER_SOLENOID);
        autoGripperDelay = new CountingDelay();
        sensorDefectCount = new CountingDelay();
    }

    @Override
    public void onStart(double timestamp) {
        zeroSensors();
        holdPosition = gripperEncoder.getPosition();

        subsystemManager.setControlState(HOLD);
//        if (DriverStation.isAutonomous()){
//            subsystemManager.setControlState(HOLD);
//        } else {
//            subsystemManager.setControlState(DISABLED);
//        }
        autoGripperDelay.reset();
    }

    @Override
    public void readPeriodicInputs() {
        // Detect object
        if (gripperMotor.getAnalog(SparkMaxAnalogSensor.Mode.kAbsolute).getVoltage() < LOWER_BOUNDARY_ANALOG_SENSOR){
            subsystemManager.setDetectObject(false);
            sensorDefectCount.reset();
        } else if (gripperMotor.getAnalog(SparkMaxAnalogSensor.Mode.kAbsolute).getVoltage() > UPPER_BOUNDARY_ANALOG_SENSOR) {
            subsystemManager.setDetectObject(true);
            sensorDefectCount.reset();
        } else if (sensorDefectCount.delay(SENSOR_DEFECT_DELAY)) {
//                subsystemManager.setSensorDisconnected(true);
            // TODO make sensor return to connected
            }
    }

    @Override
    public void onLoop(double timestamp) {
        GripperManager.ControlState controlState = subsystemManager.getControlState();
        // If game piece is cube
        if (gamePieceHandler.getGamePiece() == GamePieceHandler.GamePiece.CUBE) {
            subsystemManager.setGripperCurrentLimit(GRIPPER_CUBE_CURRENT_LIMIT);
            // Cone always open
            subsystemManager.setGripperOpen(GRIPPER_OPEN);
            switch (controlState) {
                case DISABLED:
                    // Do nothing
                    autoGripperDelay.reset();
                    subsystemManager.setGripperSpeed(0);
                    break;
                case HOLD:
                    // Keep holding
                    subsystemManager.setGripperSpeed(0);
                    if (!subsystemManager.getSensorDisconnected()
                            && !subsystemManager.getDetectObject()
                            && autoGripperDelay.delay(AUTO_CUBE_DELAY)) {
                        subsystemManager.setControlState(DISABLED);
                        autoGripperDelay.reset();
                    }
                    break;
                case SHOOT:
                    // Shoot until game piece gone
                    // TODO check if current limit is high enough for shooting
//                    subsystemManager.setGripperOpen(false);
                    subsystemManager.setGripperSpeed(GRIPPER_SHOOTSPEED);
                    if (!subsystemManager.getSensorDisconnected()
                            &&!subsystemManager.getDetectObject()
                            && autoGripperDelay.delay(AUTO_CUBE_DELAY)) {
                        subsystemManager.setControlState(DISABLED);
                        autoGripperDelay.reset();
                    }
                    break;
                case INTAKE:
                    // Intake until game piece in
                    subsystemManager.setGripperSpeed(GRIPPER_INTAKESPEED);
                    if (!subsystemManager.getSensorDisconnected()
                            && subsystemManager.getDetectObject()
                            && autoGripperDelay.delay(AUTO_CUBE_DELAY)) {
                        holdPosition = gripperEncoder.getPosition();
                        subsystemManager.setControlState(HOLD);
                        autoGripperDelay.reset();
                    }
                    break;
                case OUTTAKE:
                    // Outtake until game piece gone
                    subsystemManager.setGripperSpeed(GRIPPER_CUBE_OUTTAKESPEED);
                    if (!subsystemManager.getSensorDisconnected()
                            && !subsystemManager.getDetectObject()
                            && autoGripperDelay.delay(AUTO_CUBE_DELAY_OUTTAKE)) {
                        subsystemManager.setControlState(DISABLED);
                        autoGripperDelay.reset();
                    }
                    break;
            }
        } else {
            // If game piece is cone
            subsystemManager.setGripperCurrentLimit(GRIPPER_CONE_CURRENT_LIMIT);
            switch (controlState) {
                case DISABLED:
                    // Do nothing
                    autoGripperDelay.reset();
                    subsystemManager.setGripperOpen(GRIPPER_OPEN);
                    subsystemManager.setGripperSpeed(0);
                    break;
                case HOLD:
                    // Keep holding
                    subsystemManager.setGripperOpen(!GRIPPER_OPEN);
                    subsystemManager.setGripperSpeed(0);
                    if (!subsystemManager.getSensorDisconnected()
                            && !subsystemManager.getDetectObject()
                            && autoGripperDelay.delay(AUTO_CONE_DELAY)) {
                        subsystemManager.setControlState(DISABLED);
                        autoGripperDelay.reset();
                    }
                    break;
                case SHOOT:
                    // Shoot until game piece gone
                    subsystemManager.setGripperOpen(!GRIPPER_OPEN);
                    subsystemManager.setGripperSpeed(GRIPPER_SHOOTSPEED);
                    if (!subsystemManager.getSensorDisconnected()
                            && !subsystemManager.getDetectObject()
                            && autoGripperDelay.delay(AUTO_CONE_DELAY)) {
                        subsystemManager.setControlState(DISABLED);
                        autoGripperDelay.reset();
                    }
                    break;
                case INTAKE:
                    // Intake until game piece in
                    subsystemManager.setGripperOpen(!GRIPPER_OPEN);
                    subsystemManager.setGripperSpeed(GRIPPER_INTAKESPEED);
                    if (!subsystemManager.getSensorDisconnected()
                            && subsystemManager.getDetectObject()
                            && autoGripperDelay.delay(AUTO_CONE_DELAY)) {
                        holdPosition = gripperEncoder.getPosition();
                        subsystemManager.setControlState(HOLD);
                        autoGripperDelay.reset();
                    }
                    break;
                case OUTTAKE:
                    // Outtake until game piece gone
                    subsystemManager.setGripperOpen(GRIPPER_OPEN);
                    subsystemManager.setGripperSpeed(0);
                    if (!subsystemManager.getSensorDisconnected()
                            && !subsystemManager.getDetectObject()
                            && autoGripperDelay.delay(AUTO_CONE_DELAY)) {
                        subsystemManager.setControlState(DISABLED);
                        autoGripperDelay.reset();
                    }
                    break;
            }
        }
    }

    @Override
    public void writePeriodicOutputs() {
        // Set smart current limit
        if (prevControlState != subsystemManager.getControlState() || gamePieceHandler.getGamePiece() != prevGamePiece){
            gripperMotor.setSmartCurrentLimit(subsystemManager.getGripperCurrentLimit());
        }
        prevControlState = subsystemManager.getControlState();
        prevGamePiece = gamePieceHandler.getGamePiece();
        // Set motor state
        if (subsystemManager.getControlState() == HOLD) {
            if (enterHold){
                holdPosition = gripperEncoder.getPosition();
                enterHold = false;
            }
            gripperController.setReference(holdPosition, CANSparkMax.ControlType.kPosition, 0);
        } else {
            enterHold = true;
            gripperMotor.set(subsystemManager.getGripperSpeed());
        }
        // Set solenoid state
        gripperPiston.set(subsystemManager.getGripperOpen());
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
        gripperMotor.set(0);
        subsystemManager.setControlState(DISABLED);
    }

    @Override
    public void outputData() {
        SmartDashboard.putBoolean("gripper/objectdetect", subsystemManager.getDetectObject());
        SmartDashboard.putString("gripper/state", subsystemManager.getControlState().toString());
        SmartDashboard.putNumber("gripper/sensor value", gripperMotor.getAnalog(SparkMaxAnalogSensor.Mode.kAbsolute).getVoltage());
        SmartDashboard.putNumber("gripper/current", gripperMotor.getOutputCurrent());
        SmartDashboard.putBoolean("gripper/sensor disconnected", subsystemManager.getSensorDisconnected());
        SmartDashboard.putNumber("gripper/gripper speed", subsystemManager.getGripperSpeed());
        SmartDashboard.putNumber("gripper/gripper encoder rpm", gripperEncoder.getVelocity());
    }
}
