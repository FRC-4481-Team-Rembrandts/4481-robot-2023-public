package frc.team4481.lib.swerve;

import com.revrobotics.*;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class SwerveTurnHelper
{
    public CANSparkMax turnMotor;
    public ArmFeedforward FFController;

    /**
     *
     * @param motorID
     * @param PIDValues
     * @param FFValues
     */
    public SwerveTurnHelper(
            int motorID,
            PIDValueContainer PIDValues,
            FFValueContainer FFValues,
            int current_limit
    ) {
        turnMotor = new CANSparkMax(motorID, CANSparkMax.MotorType.kBrushless);
        turnMotor.restoreFactoryDefaults();
        turnMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        turnMotor.setSmartCurrentLimit(current_limit);

        SparkMaxAbsoluteEncoder absoluteEncoder = turnMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        absoluteEncoder.setInverted(true);

        // Convert from rotations to radians
        // And from RMP to rad/s
        absoluteEncoder.setPositionConversionFactor(2 * Math.PI);
        absoluteEncoder.setVelocityConversionFactor(2 * Math.PI / 60);

        SparkMaxPIDController controller = turnMotor.getPIDController();
        controller.setFeedbackDevice(absoluteEncoder);

        controller.setPositionPIDWrappingEnabled(true);
        controller.setPositionPIDWrappingMinInput(0);
        controller.setPositionPIDWrappingMaxInput(2 * Math.PI);
        controller.setP(PIDValues.kP);
        controller.setI(PIDValues.kI);
        controller.setD(PIDValues.kD);
        turnMotor.burnFlash();

        FFController = new ArmFeedforward(
                FFValues.kS,
                0.0,
                FFValues.kV,
                FFValues.kA
        );
    }
}
