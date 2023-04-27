package frc.team4481.robot.subsystems.modules;

import com.revrobotics.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4481.lib.subsystems.SubsystemBase;
import frc.team4481.robot.Constants;

import static frc.team4481.robot.Constants.HardwareMap.*;
import static frc.team4481.robot.Constants.Pivot.Comp.*;
import static frc.team4481.robot.Constants.kLooperDt;


public class Pivot extends SubsystemBase<PivotManager> {
    private CANSparkMax motor;
    private RelativeEncoder relativeEncoder;
    private SparkMaxAbsoluteEncoder absoluteEncoder;
    private SparkMaxPIDController pIDController;

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

    /**
     * Position the pivot must hold when it cannot move
     */
    double holdingAngle = 0;

    double targetAngle;

    public Pivot() {
        name = "Pivot";
        subsystemManager = new PivotManager();

        // Spark max
        motor = new CANSparkMax(PIVOT_MOTOR_ID, CANSparkMax.MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        motor.setSmartCurrentLimit(CURRENT_LIMIT);

        // Increase frame period for duty cycle encoder
        motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus5, 10);

        // Relative encoder
        relativeEncoder = motor.getEncoder();
        // Position in degrees
        relativeEncoder.setPositionConversionFactor(POSITION_FACTOR * GEAR_RATIO);
        // Velocity in degrees per second
        relativeEncoder.setVelocityConversionFactor(POSITION_FACTOR * GEAR_RATIO / 60);

        // Absolute encoder
        absoluteEncoder = motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        absoluteEncoder.setInverted(INVERT_ABS_ENC);
        absoluteEncoder.setPositionConversionFactor(POSITION_FACTOR);
        absoluteEncoder.setVelocityConversionFactor(POSITION_FACTOR / 60);
//        absoluteEncoder.setZeroOffset(ENCODER_OFFSET);

        // Spark max PID controller
        pIDController = motor.getPIDController();
        pIDController.setFeedbackDevice(absoluteEncoder);
        pIDController.setP(KP);
        pIDController.setI(KI);
        pIDController.setD(KD);
        pIDController.setFF(0);
//        pIDController.setIZone(ALLOWED_ERROR_PID/4);

        profileConstraints = new TrapezoidProfile.Constraints(MAX_VEL, MAX_ACCEL);

        motor.burnFlash();
    }

    @Override
    public void onStart(double timestamp) {

        holdingAngle = getAbsoluteAngle();
        setPoint = new TrapezoidProfile.State(holdingAngle, 0);
        goal = new TrapezoidProfile.State(holdingAngle,0);
        zeroSensors();

    }

    @Override
    public void onLoop(double timestamp) {
        switch (subsystemManager.getCurrentControlState()){
            case DISABLED:
                motor.set(0);
                break;
            case AUTOMATIC:
                //For the pivot, checking what the movement state is, should be done in the beginning
                //because it is needed for holding the angle if the pivot cannot move
                checkMovementState();

                //Check whether the pivot angle combined with the desired position allows for safe telescope extensions
                checkSafeZone();

                if (subsystemManager.isMovable()){
                    holdingAngle = getAbsoluteAngle();
                    profileConstraints = new TrapezoidProfile.Constraints(MAX_VEL, MAX_ACCEL);
                    setDesiredAngle(subsystemManager.getCurrentPositionState().getValue());
                } else {
                    //If the pivot is not movable, because the telescope is extended, but it is on a target
                    //move to the target angle instead of the holding angle instead
                    if (subsystemManager.getCurrentMovementState() == PivotManager.MovementState.ON_TARGET){
                        holdingAngle = subsystemManager.currentPositionState.getValue();
                    }
                    //Check if holding angle is smaller than spindexer safe angle,
                    //if that is the case, the arm can already move a bit to the highest safe position
                    if (holdingAngle <= ExtensionLimits.DEXER_EXTENSION_LIMIT){
                        holdingAngle = ExtensionLimits.DEXER_EXTENSION_LIMIT;
                        profileConstraints = new TrapezoidProfile.Constraints(MAX_VEL/10,MAX_ACCEL/10);
                    } else {
                        profileConstraints = new TrapezoidProfile.Constraints(MAX_VEL, MAX_ACCEL);
                    }
                    setDesiredAngle(holdingAngle);
                }
                break;
            case MANUAL:
                checkMovementState();
                setDesiredAngle(subsystemManager.getCurrentPositionState().getValue());
                break;
        }
        subsystemManager.setArmInRobot(getAbsoluteAngle() > LOWER_ARM_IN_ROBOT && getAbsoluteAngle() < UPPER_ARM_IN_ROBOT);
    }

    @Override
    public void onStop(double timestamp) {

    }

    @Override
    public void readPeriodicInputs() {

    }

    @Override
    public void writePeriodicOutputs() {

    }

    @Override
    public void zeroSensors() {

        relativeEncoder.setPosition(getAbsoluteAngle());

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
        SmartDashboard.putString("pivot/pos state", subsystemManager.getCurrentPositionState().toString());
        SmartDashboard.putString("pivot/mov state", subsystemManager.getCurrentMovementState().toString());
        SmartDashboard.putNumber("pivot/position", getAbsoluteAngle());
        SmartDashboard.putNumber("pivot/target position", subsystemManager.getCurrentPositionState().getValue());
        SmartDashboard.putNumber("pivot/velocity", getVelocity());
        SmartDashboard.putNumber("pivot/current", motor.getOutputCurrent());
    }

    private void setDesiredAngle(double desiredAngle) {

        double clampedDesiredAngle = MathUtil.clamp(desiredAngle, MIN_ANGLE_LIMIT, MAX_ANGLE_LIMIT);
        goal = new TrapezoidProfile.State(clampedDesiredAngle, 0);

        double dt = timer.get() - previousT;

//        double targetAngle;
        double targetVelocity;
        double targetAcceleration;

        //Check if arm is close enough to setpoint
        double error = clampedDesiredAngle - getAbsoluteAngle();
        if (Math.abs(error) <= ALLOWED_ERROR_PID){
            targetVelocity = 0;
            targetAcceleration = 0;
            targetAngle = clampedDesiredAngle;
        } else {
            // Create a motion profile with the given maximum velocity and maximum
            // acceleration constraints for the next setpoint, the desired goal, and the
            // current setpoint.
            TrapezoidProfile profile = new TrapezoidProfile(profileConstraints, goal, setPoint);
            setPoint = profile.calculate(kLooperDt);
            targetAngle = setPoint.position;
            targetVelocity = setPoint.velocity;
            targetAcceleration = (targetVelocity - prevTargetVelocity) / dt;
        }

        // Arm Feed Forward
        double currentAngle = getAbsoluteAngle();
        double arbFFComponent = KS * Math.signum(180 - currentAngle)
                + KG * Math.sin(Math.toRadians(-currentAngle))
                + KV * targetVelocity
                + KA * targetAcceleration;

        pIDController.setReference(targetAngle, CANSparkMax.ControlType.kPosition, 0, arbFFComponent);


        prevTargetVelocity = targetVelocity;
        previousT = timer.get();

        SmartDashboard.putNumber("pivot/pid target position", targetAngle);
        SmartDashboard.putNumber("pivot/target velocity", targetVelocity);
        SmartDashboard.putNumber("pivot/target acceleration", targetAcceleration);
        SmartDashboard.putNumber("pivot/arb ff", arbFFComponent);

    }

    private void checkMovementState(){

        //pivot is moving if it can move and is not in range of target
        double error = subsystemManager.getCurrentPositionState().getValue() - getAbsoluteAngle();
        SmartDashboard.putBoolean("pivot/outside target", Math.abs(error) >= ALLOWED_ERROR_STATE);

        if (Math.abs(error) >= ALLOWED_ERROR_STATE || Math.abs(getVelocity()) >= STEADY_STATE_VEL){
            subsystemManager.setCurrentMovementState(PivotManager.MovementState.MOVING);
        }
        else{
            subsystemManager.setCurrentMovementState(PivotManager.MovementState.ON_TARGET);
        }
    }

    private void checkSafeZone(){
        subsystemManager.setInSafeZone(
                (subsystemManager.getCurrentPositionState() == PivotManager.PositionState.CUBE_HIGH
                        && getAbsoluteAngle() < ExtensionLimits.CUBE_HIGH_EXTENSION_LIMIT)
                || (subsystemManager.getCurrentPositionState() == PivotManager.PositionState.CONE_HIGH
                        && getAbsoluteAngle() < ExtensionLimits.CONE_HIGH_EXTENSION_LIMIT)
        );
    }

    /**
     * @return Absolute angle of arm in range of 0 to 360 degrees with 0 down to ground and +180 up from floor
     */
    public double getAbsoluteAngle() {
        return absoluteEncoder.getPosition();
    }

    public double getAbsoluteAngleTarget() {
        return targetAngle;
    }

    /**
     * @return Relative angle of arm
     */
    private Rotation2d getRelativeAngle() {
        return Rotation2d.fromDegrees(relativeEncoder.getPosition());
    }

    /**
     * @return Velocity of arm
     */
    private double getVelocity() {
        return relativeEncoder.getVelocity();
    }

}
