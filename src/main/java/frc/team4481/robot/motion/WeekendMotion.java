package frc.team4481.robot.motion;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class WeekendMotion {

    private final double maxVelocity;
    private final double maxAcceleration;
    private final double allowedCloseLoopError;
    private double acceleration;
    private boolean decelerating;

    public WeekendMotion(double maxVelocity, double maxAcceleration, double allowedCloseLoopError) {
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.allowedCloseLoopError = allowedCloseLoopError;
        acceleration = 0;
        decelerating = false;
    }

    public double getTargetVelocity(double currentPosition, double targetPosition, double currentVelocity, double targetVelocity, double dt) {

        double pos_error = targetPosition - currentPosition;

        double direction = Math.signum(pos_error);

        //Check if we have reached desired position or something in the neighbourhood
        if (Math.abs(pos_error) < allowedCloseLoopError) {
            acceleration = 0;
            decelerating = false;
            return 0;
        }

        //Check if system is going in the right direction
        //If not, accelerate
        if ((Math.signum(currentVelocity) != direction) && (Math.signum(currentVelocity) != 0)) {
            decelerating = false;
            acceleration = clampAcceleration(maxAcceleration * direction, pos_error);
            double newTargetVelocity = targetVelocity + (acceleration * dt);
            return clampVelocity(newTargetVelocity, pos_error);
        }

        //Check if we have to decelerate to get to the target position
        double decelDistance = decelerationToDistance(targetVelocity, maxAcceleration, allowedCloseLoopError, direction);
        SmartDashboard.putNumber("decelDistance", decelDistance);

        if ((Math.abs(decelDistance) > Math.abs(pos_error))) {
            acceleration = - distanceToDeceleration(targetVelocity, pos_error) * direction;
            acceleration = clampAcceleration(acceleration, pos_error);
            decelerating = true;
            double newTargetVelocity = targetVelocity + (acceleration * dt);
            return clampVelocity(newTargetVelocity, pos_error);
        }

        //Check if we can accelerate
        if (Math.abs(targetVelocity) < maxVelocity) {
            acceleration = clampAcceleration(maxAcceleration * direction, pos_error);

            double newTargetVelocity = clampVelocity(targetVelocity + (acceleration * dt), pos_error);
            acceleration = (newTargetVelocity - targetVelocity) / dt;
            return newTargetVelocity;
        }

        acceleration = 0;
        return clampVelocity(targetVelocity, pos_error);
    }

    public double getAcceleration() {
        return acceleration;
    }

    private double clampVelocity(double velocity, double distance) {
        if (Math.abs(distance) < 0){
            return MathUtil.clamp(velocity, -10,10);
        }
        return MathUtil.clamp(velocity, -maxVelocity, maxVelocity);
    }

    private double clampAcceleration(double acceleration, double distance){
        if (Math.abs(distance) < 0){
            return MathUtil.clamp(acceleration, -10,10);
        }
        return MathUtil.clamp(acceleration, -maxAcceleration, maxAcceleration);
    }

    private double decelerationToDistance(double velocity, double acceleration, double error, double direction) {
        return (Math.pow(velocity, 2)) / (2 * acceleration * direction) + error * direction;
    }

    private double distanceToDeceleration(double velocity, double distance){
        return Math.pow(velocity,2) / (2 * Math.abs(distance));
    }

}
