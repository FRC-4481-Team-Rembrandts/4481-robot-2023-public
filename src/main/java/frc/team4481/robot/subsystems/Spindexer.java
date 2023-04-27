//package frc.team4481.robot.subsystems;
//
//import edu.wpi.first.math.MathUtil;
//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.wpilibj.AnalogInput;
//import com.revrobotics.CANSparkMax;
//import edu.wpi.first.wpilibj.DutyCycleEncoder;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.team4481.lib.subsystems.SubsystemBase;
//import frc.team4481.lib.subsystems.SubsystemHandler;
//import frc.team4481.lib.hardware.LazyCANSparkMax;
//import frc.team4481.lib.util.CountingDelay;
//import frc.team4481.robot.util.GamePieceHandler;
//
//import static frc.team4481.robot.Constants.Spindexer.Comp.*;
//import static frc.team4481.robot.Constants.HardwareMap.*;
//import static frc.team4481.robot.util.GamePieceHandler.GamePiece.*;
//
//public class Spindexer extends SubsystemBase<SpindexerManager> {
//    private final SubsystemHandler subsystemHandler = SubsystemHandler.getInstance();
//    private final GamePieceHandler gamePieceHandler = GamePieceHandler.getInstance();
//
//    private LazyCANSparkMax spindexerMotor;
//    private LazyCANSparkMax feederWheels;
//    private DutyCycleEncoder absoluteEncoder;
//    private AnalogInput occupancySensor;
//    private AnalogInput cutoutSensor;
//    private CountingDelay feederDelay;
//    private CountingDelay spinReverseDelay;
//    private CountingDelay spinDelay;
//
//    private boolean firstPass = false;
//    private double setPoint;
//    private boolean setPointEnabled = false;
//    private double prevSetpoint;
//    private boolean isClockWise;
//    private double distanceOffset;
//
//
//    public Spindexer(){
//        name = "Spindexer";
//        subsystemManager = new SpindexerManager();
//
//        spindexerMotor = new LazyCANSparkMax(TURNING_TABLE_MOTOR, SPIN_MOTOR_TYPE);
//        spindexerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
//
//        spindexerMotor.burnFlash();
//
//        absoluteEncoder = new DutyCycleEncoder(ENCODER_DIO_CHANNEL);
//
//        feederWheels = new LazyCANSparkMax(FEED_TROUGH_MOTOR, CANSparkMax.MotorType.kBrushless);
//        feederWheels.setIdleMode(CANSparkMax.IdleMode.kCoast);
//
//        occupancySensor = new AnalogInput(OCCUPANCY_SENSOR_ANALOG_CHANNEL);
//        cutoutSensor = new AnalogInput(CUTOUT_SENSOR_ANALOG_CHANNEL);
//
//        feederDelay = new CountingDelay();
//        spinDelay = new CountingDelay();
//        spinReverseDelay = new CountingDelay();
//    }
//
//    @Override
//    public void onStart(double timestamp) {
//        subsystemManager.setControlState(SpindexerManager.ControlState.DISABLED);
//
//        feederDelay.reset();
//        spinReverseDelay.reset();
//        spinDelay.reset();
//        firstPass = false;
//        zeroSensors();
//    }
//
//    @Override
//    public void readPeriodicInputs() {
//        // Check occupancy and cutout sensor
//        subsystemManager.setIsOccupied(occupancySensor.getValue() <= OCCUPANCY_SENSOR_THRESHOLD);
//        subsystemManager.setConeInCutout(cutoutSensor.getValue() >= CUTOUT_SENSOR_THRESHOLD);
//    }
//
//    @Override
//    public void onLoop(double timestamp) {
//        switch (subsystemManager.getControlState()) {
//            case DISABLED:
//                subsystemManager.setSpindexerState(SpindexerManager.DirectionState.IDLE);
//                subsystemManager.setFeederState(SpindexerManager.DirectionState.IDLE);
//                feederDelay.reset();
//                spinReverseDelay.reset();
//                spinDelay.reset();
//                break;
//            case MANUAL:
//                break;
//            case ENABLED:
//                if (gamePieceHandler.getGamePiece() == CUBE) {
//                    subsystemManager.setSpindexerState(SpindexerManager.DirectionState.IDLE);
//                    // If a cube is expected
//                    if (!subsystemManager.getIsOccupied()) {
//                        // If nothing occupies the spindexer
//                        // Start feeder wheels
//                        subsystemManager.setFeederState(SpindexerManager.DirectionState.COUNTERCLOCKWISESLOW);
//                    } else {
//                        // If something is in the spindexer
//                        if (feederDelay.delay(FEEDER_DELAY)) {
//                            // Stop feeder wheels after a delay
//                            subsystemManager.setFeederState(SpindexerManager.DirectionState.IDLE);
//                            // Stop spindexer
//                            subsystemManager.setControlState(SpindexerManager.ControlState.DONE);
//                            // Reset delay
//                            feederDelay.reset();
//                        }
//                    }
//                } else if (gamePieceHandler.getGamePiece() == CONE) {
//                    // Start feeder + spindexer
//                    subsystemManager.setFeederState(SpindexerManager.DirectionState.COUNTERCLOCKWISE);
//
//
//                    // If something in spindexer
//                    if (subsystemManager.getIsOccupied()){
//                        // Set feerder at half speed
//                        subsystemManager.setFeederState(SpindexerManager.DirectionState.COUNTERCLOCKWISESLOW);
//                        if (feederDelay.delay(0.8)) {
//                            subsystemManager.setSpindexerState(SpindexerManager.DirectionState.COUNTERCLOCKWISE);
//                        }
//                        else {
//                            firstPass=false;
//                        }
//                    } else {
//                        subsystemManager.setSpindexerState(SpindexerManager.DirectionState.CLOCKWISE);
//                        feederDelay.reset();
//                    }
//
//                    if (subsystemManager.getConeInCutout() && !firstPass) {
//                        firstPass = true;
//                        spinDelay.reset();
//                    }
//                    if (firstPass && spinDelay.delay(SPIN_DELAY)) {
//                        firstPass = false;
//                        subsystemManager.setControlState(SpindexerManager.ControlState.POSITION);
//                    }
//                }
//                break;
//
//            case POSITION:
//                subsystemManager.setSpindexerState(SpindexerManager.DirectionState.BOOST);
//                subsystemManager.setFeederState(SpindexerManager.DirectionState.IDLE);
//                if (subsystemManager.getConeInCutout() && subsystemManager.getIsOccupied()) {
//                    subsystemManager.setSpindexerState(SpindexerManager.DirectionState.IDLE);
//                    subsystemManager.setControlState(SpindexerManager.ControlState.DONE);
//                }
//                break;
//            case DONE:
//                // Spindexer done, item is aligned
//                break;
//
//            case REVERSE_FEEDER:
//                subsystemManager.setSpindexerState(SpindexerManager.DirectionState.IDLE);
//                subsystemManager.setFeederState(SpindexerManager.DirectionState.CLOCKWISE);
//                break;
//        }
//    }
//    @Override
//    public void writePeriodicOutputs() {
//        feederWheels.set(subsystemManager.getFeederState().value * FEEDER_SPEED);
//        if (subsystemManager.getSpindexerState() == SpindexerManager.DirectionState.SPEED) {
//            spindexerMotor.set(subsystemManager.getSpinSpeed());
//        }
//        else {
//            spindexerMotor.set(subsystemManager.getSpindexerState().value * SPIN_SPEED);
//        }
////        if(setPointEnabled){
//////            double prevSetpoint;
//////            if (setPoint == WEEKEND_ANGLE) {
//////                prevSetpoint = SENSOR_ANGLE;
//////            } else {
//////                prevSetpoint = WEEKEND_ANGLE;
//////            }
////            double prevError = getAngleDistance() - prevSetpoint;
////            double error = setPoint - getAngleDistance();
////            if (Math.abs(error) > Math.abs(prevError)) {
////                error = prevError;
////            }
//////            double error = setPoint - getAbsoluteAngle().getDegrees();
//////            double RAMPDOWN_THRESHOLD = 50;
////            double RAMPDOWN_THRESHOLD = 80;
////            double speed;
////            if (Math.abs(error) > RAMPDOWN_THRESHOLD) {
////                speed = (SPIN_SPEED+ KS)*Math.signum(error);
////
////
////            } else if (Math.abs(error) <= RAMPDOWN_THRESHOLD && Math.abs(error) > ANGLE_ERROR) {
////                speed = MathUtil.clamp(SPIN_SPEED*error/RAMPDOWN_THRESHOLD + Math.signum(error)* KS,-1,1);
////            } else {
////                speed = 0;
////            }
////            spindexerMotor.set(speed);
////            SmartDashboard.putNumber("Spindexer/error", error);
////            SmartDashboard.putNumber("Spindexer/given speed", speed);
////
////
////        }
////        else{
////            if (subsystemManager.getSpindexerState() == SpindexerManager.DirectionState.SPEED) {
////                spindexerMotor.set(subsystemManager.getSpinSpeed());
////            }
////            else {
////                spindexerMotor.set(subsystemManager.getSpindexerState().value * SPIN_SPEED);
////            }
////        }
//    }
//
//    @Override
//    public void onStop(double timestamp) {
//        terminate();
//    }
//
//    @Override
//    public void zeroSensors() {
//
//    }
//
//    @Override
//    public void terminate() {
//        subsystemManager.setControlState(SpindexerManager.ControlState.DISABLED);
//    }
//
//    @Override
//    public void outputData() {
////        SmartDashboard.putNumber("Spindexer/occupancy value", occupancySensor.getValue());
////        SmartDashboard.putBoolean("Spindexer/occupancy", subsystemManager.getIsOccupied());
////        SmartDashboard.putNumber("Spindexer/cutout value", cutoutSensor.getValue());
////        SmartDashboard.putBoolean("Spindexer/cutout", subsystemManager.getConeInCutout());
////        SmartDashboard.putString("Spindexer/state", subsystemManager.getControlState().toString());
////        SmartDashboard.putString("Spindexer/spin state", subsystemManager.getSpindexerState().toString());
////        SmartDashboard.putString("Spindexer/feeder state", subsystemManager.getFeederState().toString());
////        SmartDashboard.putNumber("Spindexer/spin angle", getAbsoluteAngle().getDegrees());
////        SmartDashboard.putNumber("Spindexer/raw spin angle", absoluteEncoder.getAbsolutePosition());
////        SmartDashboard.putNumber("Spindexer/spin setpoint", setPoint);
////        SmartDashboard.putBoolean("Spindexer/spin setpoint enable", setPointEnabled);
////        SmartDashboard.putNumber("Spindexer/spin setpoint prev", prevSetpoint);
////        SmartDashboard.putNumber("Spindexer/spin speed", subsystemManager.getSpinSpeed());
////        SmartDashboard.putNumber("Spindexer/raw distance", absoluteEncoder.getDistance());
////        SmartDashboard.putNumber("Spindexer/encoder distance", getAngleDistance());
//    }
//
//    private Rotation2d getAbsoluteAngle() {
//        double rotationDegrees = 360 - (absoluteEncoder.getAbsolutePosition() - ENCODER_OFFSET) * 360;
//        return Rotation2d.fromDegrees(MathUtil.inputModulus(rotationDegrees,0,360));
//    }
//
//    private double getAngleDistance() {
//        return 360 - (absoluteEncoder.getDistance() - ENCODER_OFFSET) * 360;
//    }
//
//    private double getSetpoint(double targetAngle, boolean isClockWise) {
//        double absAngle = getAbsoluteAngle().getDegrees();
//        double modAngle = getAngleDistance() - absAngle;
//
//        if (isClockWise) {
//            if (absAngle < targetAngle) {
//                return modAngle + targetAngle;
//            } else {
//                return modAngle + 360 + targetAngle;
//            }
//        } else {
//            if (absAngle > targetAngle) {
//                return  modAngle + targetAngle;
//            } else {
//                return  modAngle - 360 + targetAngle;
//            }
//        }
//
//    }
//
//}
