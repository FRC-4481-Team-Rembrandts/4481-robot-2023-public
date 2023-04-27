package frc.team4481.robot;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import static frc.team4481.robot.Constants.Drivetrain.DRIVETRAIN_WIDTH;

public class Constants {
    /* ---------------------------------------- */
    /* LOOPER */
    /* ---------------------------------------- */
    public static final double kLooperDt = 0.02;
    public static double kHIDLooperDt = 0.02;

    /**
     * Static class that holds all the CAN IDs and IO port addresses on the RoboRIO
     */
    public static class HardwareMap {
        // Sensors
        public static final int PIGEON_IMU = 9;

        // Drivetrain
        public static final int DT_FRONT_LEFT_DRIVE_ID = 11;
        public static final int DT_FRONT_LEFT_TURN_ID = 12;
        public static final boolean DT_FRONT_LEFT_INVERTED = true;

        public static final int DT_FRONT_RIGHT_DRIVE_ID = 13;
        public static final int DT_FRONT_RIGHT_TURN_ID = 14;
        public static final boolean DT_FRONT_RIGHT_INVERTED = false;

        public static final int DT_BACK_LEFT_DRIVE_ID = 15;
        public static final int DT_BACK_LEFT_TURN_ID = 16;
        public static final boolean DT_BACK_LEFT_INVERTED = true;

        public static final int DT_BACK_RIGHT_DRIVE_ID = 17;
        public static final int DT_BACK_RIGHT_TURN_ID = 18;
        public static final boolean DT_BACK_RIGHT_INVERTED = false;

        public static final int LED_CONTROLLER = 9;

        // Pivot
        public static final int PIVOT_MOTOR_ID = 41;

        // Telescope
        public static final int TELESCOPE_MOTOR_ID = 42;

        // Gripper
        public static final int GRIPPER_MOTOR = 51;


        // Spindexer
        public static final int OCCUPANCY_SENSOR_ANALOG_CHANNEL = 1;

        // Intake
        public static final int INTAKE_MOTOR = 31;

        public static class Comp {
            public static final int INTAKE_SOLENOID = 2;
            public static final int GRIPPER_SOLENOID = 1;
        }

        // Pneumatics
        public static final int COMPRESSOR_RELAY_CHANNEL = 0;
        public static final int PRESSURE_ANALOG_CHANNEL = 0;
    }

    /**
     * Static class containing all the constants for the drivetrain subsystem
     */
    public static class Drivetrain {
        /**
         * 1-dimensional distance between the center of a wheel and the center of the drivetrain in m
         */
        public static final double DRIVETRAIN_WHEELBASE_DISTANCE = 0.285;
        public static final double DRIVE_GEAR_RATIO = 4.71;
        public static final double WHEEL_RADIUS = 0.0762 / 2;
        public static final double DRIVETRAIN_WIDTH = 0.82;
        public static final double TURN_GEAR_RATIO = 2.89 * 3.61 /14 * 62;
        public static final int STALL_LIMIT_DRIVE = 60; //Amps
        public static final int STALL_LIMIT_DRIVE_CHARGE = 40;//46; //Amps
        public static final int STALL_LIMIT_TURN = 23; //in Amps
        public static final double WALL_ANGLE_MARGIN = 10;

        /**
         *  Maximum limit for the robot heading slew rate limiter in DEG/S
         *  The used limit depends on the current speed of the robot
         */
        public static final double MAX_DIRECTION_RATE_LIMIT = 300;

        /**
         * Margin that the heading correction algorithm has in degrees
         */
        public static final double TURNING_DEADBAND = 2;
        public static final double HEADING_kP = 0.04; //P constant for heading correction

        /**
         * Drive motor RPM to wheel velocity in m/s
         */
        public static final double DRIVE_VELOCITY_CONVERSION_FACTOR = (2 * Math.PI * WHEEL_RADIUS)/(DRIVE_GEAR_RATIO * 60);
        /**
         * Drive motor revolutions to distance in m
         */
        public static final double DRIVE_POSITION_CONVERSION_FACTOR = (2 * Math.PI * WHEEL_RADIUS)/(DRIVE_GEAR_RATIO);
        /**
         * Turn motor RPM to module angular velocity in rad/s
         */
        public static final double TURN_VELOCITY_CONVERSION_FACTOR = 2 * Math.PI / (TURN_GEAR_RATIO * 60);
        /**
         * Turn motor rotations to radians
         */
        public static final double TURN_POSITION_CONVERSION_FACTOR = 2 * Math.PI / TURN_GEAR_RATIO;

        public static class AutoAlign {
            public static final double ALLOWED_ERROR_X = 0.03;
            public static final double ALLOWED_ERROR_Y = 0.007;
            public static final double kP_X = 4;
            public static final double kP_Y = 4;
            public static final double MAX_VELOCITY = 1;
            public static final double ARM_IN_X_OFFSET = 0.41;
            // TODO find good values
            public static final double CUBE_HIGH_OFFSET = 0.05; //m
            public static final double CONE_HIGH_OFFSET = -0.03; //m
            public static final double CONE_MID_OFFSET = 0.1; //m
        }

        // PIDF values. FF constants come from SysID

        // TODO sysid drivetrain values
        public static class Comp {
            public static final double DRIVE_kP = 0.23412;
            public static final double DRIVE_kI = 0;
            public static final double DRIVE_kD = 0;

            public static final double DRIVE_kA = 0.44218;
            public static final double DRIVE_kS = 0.17491;
            public static final double DRIVE_kV = 2.7538;

            public static final double TURN_kA = 0;
            public static final double TURN_kS = 0.2;
            public static final double TURN_kV = 0.6;

            public static final double TURN_kP = 1;
            public static final double TURN_kI = 0;
            public static final double TURN_kD = 0.4;
        }

        public static final CANSparkMax.IdleMode DRIVE_IDLE_MODE = CANSparkMax.IdleMode.kBrake;

        public static final Pose2d DRIVETRAIN_START_POSITION_BLUE = new Pose2d();
        public static final Pose2d DRIVETRAIN_START_POSITION_RED = new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180));
    }

    /**
     * Static class containing important field dimensions
     */
    public static class Field {
        public static final double FIELD_WIDTH = 8.20;
        public static final double FIELD_LENGTH = 16.5;
        /**
         * y-coordinate of all the nodes on the field measured from the bottom
         */
        public static final double[] NODES_Y_BLUE = {
                0.50, 1.07, 1.63, 2.24, 2.75, 3.31, 3.86, 4.43, 5.05
        };
        public static final double[] NODES_Y_RED = {
                0.50, 1.07, 1.63, 2.24, 2.75, 3.31, 3.86, 4.43, 5.05
        };
        public static final double[] CUBE_NODES_Y_BLUE = {
                1.07, 2.75,4.43
        };
        public static final double[] CUBE_NODES_Y_RED = {
                1.07, 2.75, 4.43
        };
//        public static final double[] CONE_NODES_Y_BLUE = {
//                0.50, 1.63, 2.24, 3.31, 3.93, 5.05
//        };
        public static final double[] CONE_NODES_Y_BLUE = {
                0.506, 1.629, 2.191, 3.309, 3.871, 4.994
        };
        public static final double[] CONE_NODES_Y_RED = {
                0.506, 1.629, 2.191, 3.309, 3.871, 4.994
        };
//
//        public static final double[] CONE_NODES_Y_RED = {
//                0.47, 1.61, 2.23, 3.31, 3.89, 4.96
//        };

        public static final double[] HUMAN_PLAYER_Y = {
                6, 7.5
        };

        public static final double HUMAN_PLAYER_X_RED = 1;
        public static final double HUMAND_PLAYER_X_BLUE = FIELD_LENGTH - HUMAN_PLAYER_X_RED;

        /**
         * x-coordinate to put the robot against the nodes for blue
         */
        public static final double NODE_X_BLUE = 1.27 + DRIVETRAIN_WIDTH/2;
        /**
         * x-coordinate to put the robot against the nodes for red
         */
        public static final double NODE_X_RED = FIELD_LENGTH - NODE_X_BLUE;
        /**
         * Distance between nodes in which the robot should
         * not go left or right, but stand still
         */
        public static final double NODE_DEADZONE = 0.05;
        /**
         * Max distance from the grid for auto align to activate
         */
        public static final double MAX_GRID_DISTANCE = 4;
    }

    /**
     * Static class containing all the constants for the pivot subsystem
     */
    public static class Pivot {
        // Absolute encoder offset such that 0 is parallel with floor

        public static class Comp {
            public static final boolean INVERT_ABS_ENC = true;
            public static final double GEAR_RATIO = 1/122.3272727;
            public static final double POSITION_FACTOR = 360;
            public static final double MIN_ANGLE_LIMIT = 70; // In degrees
            public static final double MAX_ANGLE_LIMIT = 280; // In degrees
            public static final double MIN_VEL = 0; //DEG/S
            public static final double MAX_VEL = 300; //DEG/S
            public static final double MAX_ACCEL = 2000; //1000; //DEG/S^2
            public static final double ALLOWED_ERROR_PID = 1; //DEG
            public static final double KP = 0.02;
            public static final double KI = 0;
            public static final double KD = 0;
            public static final double KS = 0.05;
            public static final double KG = 0.5;
            public static final double KV = 0.05;
            public static final double KA = 0.001;
            public static final double STEADY_STATE_VEL = 15;
            public static final double ALLOWED_ERROR_STATE = 3.0; // 2.0
            public static final int CURRENT_LIMIT = 40; //in Amps
            public static final double UPPER_ARM_IN_ROBOT = 260;
            public static final double LOWER_ARM_IN_ROBOT = 165;

            public static class Positions {
                public static final double TODO = 200;
                public static final double CUBE_LOW_POS = 155;
                public static final double CONE_LOW_POS = 151;
                public static final double CUBE_MID_POS = 108;
                public static final double CUBE_HIGH_POS = 90;
                public static final double CONE_MID_POS = 94;
                public static final double CONE_HIGH_POS = 78;//79;
                public static final double CONE_DEXER_POS = 197;
                public static final double CUBE_DEXER_POS = 199.5;
                public static final double CUBE_PLAYER_POS = 94;
                public static final double CONE_PLAYER_POS = 97;
                public static final double IDLE_POS = 192;//180;
                public static final double CONE_GROUND_POS = CONE_LOW_POS;
                public static final double CUBE_GROUND_POS = CUBE_LOW_POS;
                public static final double CUBE_INTAKE_POS = 212;//212;

            }
            public static class ExtensionLimits{
                public static final double CUBE_HIGH_EXTENSION_LIMIT = 128.2;
                public static final double CONE_HIGH_EXTENSION_LIMIT = 125;
                public static final double DEXER_EXTENSION_LIMIT = 95;
            }
        }
    }

    /**
     * Static class containing all the constants for the telescope subsystem
     */
    public static class Telescope {

        public static class Comp {
            public static final boolean MOTOR_INVERT = false;
            public static final double GEAR_RATIO = 16.0/80; // TODO
            public static final double POSITION_FACTOR = 9; // To get centimeter of output shaft
            public static final double MAX_EXTEND = 50;
            public static final double MIN_EXTEND = 0;
            public static final double MIN_VEL = 0; //CM/S
            public static final double MAX_VEL = 170; //170; //CM/S
            public static final double MAX_ACCEL = 1600; //600; //CM/S^2
            public static final double ALLOWED_ERROR_PID = 0.1; //CM
            public static final double KP = 0.085;
            public static final double KI = 0;
            public static final double KD = 0;
            public static final double KS = 0;
            public static final double KV = 0.08; //0.3;
            public static final double KA = 0.0022;
            public static final double ALLOWED_ERROR_STATE = 3.0;
            public static final int CURRENT_LIMIT = 50; //AMPS
            public static final double CALIBRATION_SPEED = -0.2; // Speed between -1 and 1
            public static final int CALIBRATION_CURRENT_LIMIT = 30; //AMPS
            public static final double CALIBRATION_CURRENT_MARGIN_FACTOR = 1; //Factor by which the current can exceed the limit before the calibration is finished
            public static final double CALIBRATION_RAMP_RATE = 8; //Seconds from 0 to full throttle
            public static final double STROEF_EXTENSION = 32;
            public static final double STROEF_KS = 12;

            public static final class Positions {
                public static final double CUBE_HIGH_POS = 25;
                public static final double CONE_MID_POS = 3;
                public static final double CONE_HIGH_POS = 50; // TODO find more reliable value
                public static final double CONE_DEXER_POS = 25.4;
                public static final double CUBE_DEXER_POS = 17.3;
                public static final double CONE_GROUND_POS = 34;
                public static final double CUBE_GROUND_POS = 31;
                public static final double CONE_SMASH_POS = 10;
                public static final double CUBE_INTAKE_POS = 25;//18;
            }
        }
        /**
         * Static class containing all the positions of the telescopic arm
         */

    }

    public static class Gripper {
        public static final double GRIPPER_INTAKESPEED = -0.5;
        public static final double GRIPPER_SHOOTSPEED = 0.5;
        public static final double GRIPPER_CUBE_OUTTAKESPEED = 0.3;
        public static final int GRIPPER_CUBE_CURRENT_LIMIT = 25;
        public static final int GRIPPER_CONE_CURRENT_LIMIT = 60;
        public static final double UPPER_BOUNDARY_ANALOG_SENSOR = 3.2;
        public static final double LOWER_BOUNDARY_ANALOG_SENSOR = 0.1;
        public static final double SENSOR_DEFECT_DELAY = 2;
        public static final double AUTO_CONE_DELAY = 0.2;
        public static final double AUTO_CUBE_DELAY = 0.0001;
        public static final double AUTO_CUBE_DELAY_OUTTAKE = 0.1;

        public static class Comp {
            public static final boolean GRIPPER_OPEN = true;
        }

    }

    /* ---------------------------------------- */
    /* PATH PLANNING */
    /* ---------------------------------------- */
    /*
     * Maximum robot acceleration in m/s^2
     * Maximum robot velocity in m/s
     * Minimum robot velocity in m/s
     */
    public static final double MAX_ACCELERATION = 5;
    public static final double MAX_SLOW_VELOCITY = 2;
    public static final double MAX_VELOCITY = 4.5;
    public static final double MIN_MAX_VELOCITY = 1.5;
    public static final double MIN_VELOCITY = 0.6;
    public static final double MAX_TURN_VELOCITY = 2*Math.PI;
    public static final double ROTATION_KP = 0.35;
    public static final double AUTO_BALANCE_VELOCITY = 0.5;
    public static final double AUTO_BALANCE_ANGLE_OFFSET = 5;
    public static final double TILT_THRESHOLD = 10;
    public static final double AUTO_BALANCE_KP = 0.04;//0.055;
    public static final double AUTO_BALANCE_MAX_STOP_ERROR = 2;
    public static final double CHARGE_VELOCITY = 4.5; //Velocity with which the robot goes onto the charge station

    /*
     * Maximum distance from final point that counts as ended path in m
     */
    public static final double PATH_END_DISTANCE = 0.3;

    /*
     * Minimum lookahead distance in m
     * Maximum lookahead distance in m
     */
    public static final double MIN_LOOKAHEAD = .4;
    public static final double MAX_LOOKAHEAD = .5;

    /* ---------------------------------------- */
    /* CONTROLLERS */
    /* ---------------------------------------- */
    public static final double ORANGE_LEFTSTICK_DEADBAND = 0.15;
    public static final double ORANGE_RIGHTSTICK_DEADBAND = 0.15;

    /*
    public static class Spindexer {

        public static class Comp {
            public static final CANSparkMax.MotorType SPIN_MOTOR_TYPE = CANSparkMaxLowLevel.MotorType.kBrushed;
            public static final double SPIN_SPEED = 0.5;
            public static final double SPIN_GEAR_RATIO = 1.0/150;
            public static final double SPIN_DELAY = 0.9; // seconds
            public static final double SPIN_REVERSE_DELAY = 1.5; // seconds
            public static final double FEEDER_SPEED = 0.5;
            public static final double FEEDER_DELAY = 1; // seconds
            // TODO find value such that 0 is with cutout towards intake
            public static final double ENCODER_OFFSET = 0.69 ;//0.533 ; // rev
            public static final double OCCUPANCY_SENSOR_THRESHOLD = 10;
            public static final double CUTOUT_SENSOR_THRESHOLD = 10;
            public static final double SENSOR_ANGLE = -180; // 122;
            public static final double WEEKEND_ANGLE = 190;
            public static final double ANGLE_ERROR = 2;
            public static final double KS = 0.1;
        }
    }
    */

    public static class Pneumatics {
        public static final double PRESSURE_THRESHOLD_DISABLE_MATCH = 105; // Psi
        public static final double PRESSURE_THRESHOLD_ENABLE_MATCH = 70; // Psi
        public static final double PRESSURE_THRESHOLD_DISABLE_QUEUE = 119; // Psi
        public static final double PRESSURE_THRESHOLD_ENABLE_QUEUE = 90; // Psi
    }

    /**
     * Static class that contains constants for the intake subsystem
     */
    public static class Intake {
        public static final double CUBE_SPEED = 0.6; //0.5;
        public static final double CONE_SPEED = 0.5;//1;
        public static final double CONE_SPEED_REV = -0.5;
        public static final boolean INVERT_MOTOR = true;
        public static final boolean INTAKE_OPEN = true;
        public static final double PASSIVE_ROLL_TIME = 1;
        public static final double OCCUPANCY_SENSOR_THRESHOLD = 10;
        public static final double OCCUPANCY_SENSOR_DELAY = 0.2;
    }
}