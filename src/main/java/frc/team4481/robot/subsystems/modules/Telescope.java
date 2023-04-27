package frc.team4481.robot.subsystems.modules;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4481.lib.subsystems.SubsystemBase;
import frc.team4481.lib.subsystems.SubsystemHandler;

import static frc.team4481.robot.Constants.Telescope.Comp.*;
import static frc.team4481.robot.Constants.HardwareMap.*;
import static frc.team4481.robot.Constants.kLooperDt;

public class Telescope extends SubsystemBase<TelescopeManager> {
    private final SubsystemHandler subsystemHandler = SubsystemHandler.getInstance();

    private final CANSparkMax motor;
    private final RelativeEncoder relativeEncoder;
    private final SparkMaxPIDController pIDController;

    private TrapezoidProfile.Constraints profileConstraints;
    /**
     * Desired goal of the pivot.
     * The goal will be achieved by following the sequence of setpoints
     */
    private TrapezoidProfile.State goal;
    /**
     *  Current setpoint the pivot has to achieve in the next loop
     */
    private TrapezoidProfile.State setPoint;

    //Timer used to calculate derivatives
    Timer timer = new Timer();
    double previousT;

    //Variables for the Motion Profile
    double prevTargetVelocity = 0;

    private double targetPosition;


    public Telescope(){
        name = "Telescope";
        subsystemManager = new TelescopeManager();

        // Spark max
        motor = new CANSparkMax(TELESCOPE_MOTOR_ID, CANSparkMax.MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setInverted(MOTOR_INVERT);
        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        motor.setSmartCurrentLimit(CURRENT_LIMIT);

        // Relative encoder
        relativeEncoder = motor.getEncoder();
        relativeEncoder.setPositionConversionFactor(POSITION_FACTOR * GEAR_RATIO);
        relativeEncoder.setVelocityConversionFactor(POSITION_FACTOR * GEAR_RATIO / 60);

        // Spark max PID controller
        pIDController = motor.getPIDController();
        pIDController.setFeedbackDevice(relativeEncoder);
        pIDController.setSmartMotionMaxVelocity(MAX_VEL, 0);
        pIDController.setSmartMotionMinOutputVelocity(MIN_VEL, 0);
        pIDController.setSmartMotionMaxAccel(MAX_ACCEL, 0);
        pIDController.setSmartMotionAllowedClosedLoopError(ALLOWED_ERROR_PID, 0);
        pIDController.setP(KP);
        pIDController.setI(KI);
        pIDController.setD(KD);
        pIDController.setFF(0);
        motor.setOpenLoopRampRate(CALIBRATION_RAMP_RATE);

        motor.burnFlash();

        profileConstraints = new TrapezoidProfile.Constraints(MAX_VEL, MAX_ACCEL);

    }

    @Override
    public void onStart(double timestamp) {

    }

    @Override
    public void onLoop(double timestamp) {
        switch (subsystemManager.getCurrentControlState()){
            case DISABLED:
                motor.set(0);
                break;
            case CALIBRATING:
                subsystemManager.setCurrentMovementState(TelescopeManager.MovementState.MOVING);
                motor.setSmartCurrentLimit(CALIBRATION_CURRENT_LIMIT);
                motor.set(CALIBRATION_SPEED);

                if (motor.getOutputCurrent() >= CALIBRATION_CURRENT_MARGIN_FACTOR * CALIBRATION_CURRENT_LIMIT) {
                    motor.set(0);
                    subsystemManager.setCalibrated(true);
                    zeroSensors();
                }
                break;
            case AUTOMATIC:
                motor.setSmartCurrentLimit(CURRENT_LIMIT);
                if (subsystemManager.isMovable()){
                    subsystemManager.setCurrentExtensionState(TelescopeManager.ExtensionState.EXTENDED);
                    setDesiredPosition(subsystemManager.getCurrentPositionState().getValue());
                } else {
                    subsystemManager.setCurrentExtensionState(TelescopeManager.ExtensionState.RETRACTED);
                    setDesiredPosition(TelescopeManager.PositionState.RETRACTED.getValue());
                }

                //For the telescope, checking the movement state should be done at the end
                //because the extension state can be changed above
                checkMovementState();
                break;
            case MANUAL:
                checkMovementState();
                setDesiredPosition(subsystemManager.getCurrentPositionState().getValue());
                break;
        }
    }

    @Override
    public void onStop(double timestamp) {
        motor.set(0);
    }

    @Override
    public void readPeriodicInputs() {

    }

    @Override
    public void writePeriodicOutputs() {

    }

    @Override
    public void zeroSensors() {

        relativeEncoder.setPosition(0);

        setPoint = new TrapezoidProfile.State(0, 0);
        goal = new TrapezoidProfile.State(0, 0);

        timer.reset();
        timer.start();
        previousT = 0;

        prevTargetVelocity = 0;
    }

    @Override
    public void terminate() {
        motor.set(0);
    }

    @Override
    public void outputData() {
        SmartDashboard.putString("telescope/ctrl state", subsystemManager.getCurrentControlState().toString());
        SmartDashboard.putString("telescope/pos state", subsystemManager.getCurrentPositionState().toString());
        SmartDashboard.putString("telescope/move state", subsystemManager.getCurrentMovementState().toString());
        SmartDashboard.putNumber("telescope/position", relativeEncoder.getPosition());
        SmartDashboard.putNumber("telescope/velocity", relativeEncoder.getVelocity());
        SmartDashboard.putNumber("telescope/current (A)", motor.getOutputCurrent());
        SmartDashboard.putBoolean("telescope/calibrated", subsystemManager.isCalibrated());
        SmartDashboard.putString("telescope/extent state", subsystemManager.getCurrentExtensionState().toString());
    }

    private void setDesiredPosition(double desiredPosition) {
        double clampedDesiredPosition = MathUtil.clamp(desiredPosition, MIN_EXTEND, MAX_EXTEND);
        goal = new TrapezoidProfile.State(clampedDesiredPosition, 0);

        double dt = timer.get() - previousT;

//        double targetPosition;
        double targetVelocity;
        double targetAcceleration;

        double currentPosition = relativeEncoder.getPosition();
        double error = clampedDesiredPosition - currentPosition;
        if (Math.abs(error) <= ALLOWED_ERROR_PID) {
            targetPosition = clampedDesiredPosition;
            targetVelocity = 0;
            targetAcceleration = 0;
        } else {
            TrapezoidProfile profile = new TrapezoidProfile(profileConstraints, goal, setPoint);
            setPoint = profile.calculate(kLooperDt);
            targetPosition = setPoint.position;
            targetVelocity = setPoint.velocity;
            targetAcceleration = (targetVelocity - prevTargetVelocity) / dt;
        }
        // Maybe add kG
        double moveDirection = Math.signum(targetPosition - currentPosition);
        double arbFFComponent = KS * moveDirection
                + KV * targetVelocity
                + KA * targetAcceleration;

        if (currentPosition >= STROEF_EXTENSION && moveDirection < 0) {
            arbFFComponent += STROEF_KS * moveDirection;
        }


        //Apply setpoint
        pIDController.setReference(targetPosition, CANSparkMax.ControlType.kPosition,0,arbFFComponent);

        prevTargetVelocity = targetVelocity;
        previousT = timer.get();

        SmartDashboard.putNumber("telescope/pid target position",  targetPosition);
        SmartDashboard.putNumber("telescope/target velocity", targetVelocity);
        SmartDashboard.putNumber("telescope/target acceleration", targetAcceleration);
        SmartDashboard.putNumber("telescope/arb ff", arbFFComponent);
    }

    private void checkMovementState(){
        //pivot is moving if it can move and is not in range of target
        double target;
        if (subsystemManager.getCurrentExtensionState() == TelescopeManager.ExtensionState.RETRACTED){
            target = TelescopeManager.PositionState.RETRACTED.getValue();
        } else {
            target = subsystemManager.getCurrentPositionState().getValue();
        }
        double error = target - relativeEncoder.getPosition();

        if (Math.abs(error) >= ALLOWED_ERROR_STATE){
            subsystemManager.setCurrentMovementState(TelescopeManager.MovementState.MOVING);
        }else{
            subsystemManager.setCurrentMovementState(TelescopeManager.MovementState.ON_TARGET);
        }
    }

    public double getPositionCurrent() {
        return relativeEncoder.getPosition();
    }

    public double getPositionDesired() {
        return targetPosition;
    }
}
