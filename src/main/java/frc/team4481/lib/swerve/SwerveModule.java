package frc.team4481.lib.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule
{
    // Driving
    private CANSparkMax driveMotor;
    private RelativeEncoder driveEncoder;
    private SparkMaxPIDController drivePIDController;
    private SimpleMotorFeedforward driveFFController;

    // Turning
    private CANSparkMax turnMotor;
    private SparkMaxPIDController turnPIDController;

    private ArmFeedforward turnFFController;

    private SwerveModulePhysicalConstants consts;

    // Current limits
    private int stallLimitDrive;
    private int freeLimitDrive;
    private int stallLimitTurn;
    private int freeLimitTurn;

    public SwerveModule(
            SwerveTurnHelper turnHelper,
            SwerveDriveHelper driveHelper
    )
    {
        // Set turn stuff
        turnMotor = turnHelper.turnMotor;
        turnPIDController = turnMotor.getPIDController();
        turnFFController = turnHelper.FFController;

        // Set drive stuff
        driveMotor = driveHelper.driveMotor;
        driveEncoder = driveMotor.getEncoder();
        drivePIDController = driveMotor.getPIDController();
        driveFFController = driveHelper.FFController;
    }

    /**
     * Gets the current {@code SwerveModuleState} of this module
     *
     * @return current state of this module
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), getAbsoluteAngle());
    }

    /**
     * Sets the desired velocity and angle of this swerve module
     *
     * @param desiredState the desired {@code SwerveModuleState} of this module
     */
    public void setDesiredState(SwerveModuleState desiredState, double desiredTurnSpeed)
    {
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001)
        {
            stopModule();
            return;
        }
        // Optimize the desired state such that the wheel has to turn the smallest possible angle
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, getAbsoluteAngle());

        // Turning
        double targetAngle = optimizedState.angle.getRadians();

        final double turnArbFFComponent = turnFFController.calculate(targetAngle, desiredTurnSpeed);
        turnPIDController.setReference(
                targetAngle,
                CANSparkMax.ControlType.kPosition,
                0,
                turnArbFFComponent,
                SparkMaxPIDController.ArbFFUnits.kVoltage
        );
        SmartDashboard.putNumber("DT/Target angle/" + turnMotor.getDeviceId(), targetAngle);


        // Driving
        double targetVelocity = optimizedState.speedMetersPerSecond;

        double arbFFComponent = driveFFController.calculate(targetVelocity);
        drivePIDController.setReference(
                targetVelocity,
                CANSparkMax.ControlType.kVelocity,
                0,
                arbFFComponent,
                SparkMaxPIDController.ArbFFUnits.kVoltage
        );
        SmartDashboard.putNumber("DT/Target velocity/" + driveMotor.getDeviceId(), targetVelocity);
        SmartDashboard.putNumber("DT/velocity arb ff/" + driveMotor.getDeviceId(), arbFFComponent);
        SmartDashboard.putNumber("DT/applied output/" + driveMotor.getDeviceId(), driveMotor.getAppliedOutput());
    }

    /**
     * Gets the current absolute angle of the swerve module
     *
     * @return current {@code Rotation2d} of absolute turning angle
     */
    public Rotation2d getAbsoluteAngle()
    {
        SparkMaxAbsoluteEncoder encoder = turnMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

        return new Rotation2d(encoder.getPosition());
    }

    /**
     * Gets the current velocity of the swerve module in m/s.
     *
     * @return current velocity in m/s
     */
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                driveEncoder.getPosition(),
                getAbsoluteAngle()
        );
    }

    /**
     * Set both motors of o {@code SwerveModule} to 0 output.
     */
    private void stopModule()
    {
        driveMotor.set(0);
        turnMotor.set(0);
    }

    // Methods to edit Swerve module parameters
    public void setInverted(boolean isInverted) {
        driveMotor.setInverted(isInverted);
    }

    public boolean getInverted() {
        return driveMotor.getInverted();
    }

    public void setIdleMode(CANSparkMax.IdleMode mode) {
        driveMotor.setIdleMode(mode);
    }

    public CANSparkMax.IdleMode getIdleMode() {
        return driveMotor.getIdleMode();
    }

    public void setClosedLoopRampRate(double rate) {
        driveMotor.setClosedLoopRampRate(rate);
    }

    public double getClosedLoopRampRate() {
        return driveMotor.getClosedLoopRampRate();
    }

    public void setDriveCurrentLimit(int limit){
        driveMotor.setSmartCurrentLimit(limit);
    }

   }