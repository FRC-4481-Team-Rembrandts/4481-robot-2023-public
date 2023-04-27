package frc.team4481.robot.path;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4481.robot.Constants;
import frc.team4481.robot.subsystems.ArmManager;
import frc.team4481.robot.util.GamePieceHandler;

import static frc.team4481.robot.Constants.Field.*;

public class AutoAligner {
    /**
     * Gets the closest node to the current position of the robot.
     *
     * @param currentPose current {@code Pose2d} of the robot
     * @param maxGridDistance maximum distance in m from the grid that still allows for auto aligning
     * @param nodeDeadZone dead zone between nodes
     * @param armState state in which the arm is in
     * @return {@code Pose2d} of the closest node
     */
    public static Pose2d getTargetPose2d(Pose2d currentPose, double maxGridDistance, double nodeDeadZone, ArmManager.PositionState armState) {
        double currentY = currentPose.getY();
        double currentX = currentPose.getX();

        boolean isBlueAlliance = DriverStation.getAlliance() == DriverStation.Alliance.Blue;
        // Do not auto align if the robot is out of bounds
        if (
                (isBlueAlliance && currentX > maxGridDistance) ||
                (!isBlueAlliance && currentX < FIELD_LENGTH - maxGridDistance)
        ) {
            SmartDashboard.putBoolean("DT/Auto align/Auto align possible", false);
            return currentPose;
        } else {
            SmartDashboard.putBoolean("DT/Auto align/Auto align possible", true);
        }

        double[] nodes;
        double node_x;
        double angle;
        if (GamePieceHandler.getInstance().getGamePiece() == GamePieceHandler.GamePiece.CUBE) {
            if (!isBlueAlliance){
                nodes = CUBE_NODES_Y_RED;
                node_x = NODE_X_RED - Constants.Drivetrain.AutoAlign.ARM_IN_X_OFFSET/2;
                angle = 180;
                if (armState == ArmManager.PositionState.SPINDEXER_RETRACTED) {
                    node_x -= Constants.Drivetrain.AutoAlign.ARM_IN_X_OFFSET/2;
                } else if (armState == ArmManager.PositionState.HIGH) {
                    node_x += Constants.Drivetrain.AutoAlign.CUBE_HIGH_OFFSET;
                }
            } else {
                nodes = CUBE_NODES_Y_BLUE;
                node_x = NODE_X_BLUE + Constants.Drivetrain.AutoAlign.ARM_IN_X_OFFSET/2;;
                angle = 0;
                if (armState == ArmManager.PositionState.SPINDEXER_RETRACTED) {
                    node_x += Constants.Drivetrain.AutoAlign.ARM_IN_X_OFFSET/2;
                } else if (armState == ArmManager.PositionState.HIGH) {
                    node_x -= Constants.Drivetrain.AutoAlign.CUBE_HIGH_OFFSET;
                }
            }
        } else {
            if (!isBlueAlliance){
                nodes = CONE_NODES_Y_RED;
                node_x = NODE_X_RED;
                angle = 180;
                if (armState == ArmManager.PositionState.SPINDEXER_RETRACTED) {
                    node_x -= Constants.Drivetrain.AutoAlign.ARM_IN_X_OFFSET;
                } else if (armState == ArmManager.PositionState.HIGH) {
                    node_x += Constants.Drivetrain.AutoAlign.CONE_HIGH_OFFSET;
                } else if (armState == ArmManager.PositionState.MIDDLE) {
                    node_x += Constants.Drivetrain.AutoAlign.CONE_MID_OFFSET;
                }
            } else {
                nodes = CONE_NODES_Y_BLUE;
                node_x = NODE_X_BLUE;
                angle = 0;
                if (armState == ArmManager.PositionState.SPINDEXER_RETRACTED) {
                    node_x += Constants.Drivetrain.AutoAlign.ARM_IN_X_OFFSET;
                } else if (armState == ArmManager.PositionState.HIGH) {
                    node_x -= Constants.Drivetrain.AutoAlign.CONE_HIGH_OFFSET;
                } else if (armState == ArmManager.PositionState.MIDDLE) {
                    node_x -= Constants.Drivetrain.AutoAlign.CONE_MID_OFFSET;
                }
            }
        }
        if (armState != ArmManager.PositionState.SPINDEXER_RETRACTED) {
            // Make snapshot if aligning and using arm
//            NetworkTableInstance.getDefault().getTable("limelight").getEntry("snapshot").setNumber(1);
//            NetworkTableInstance.getDefault().getTable("limelight-bottom").getEntry("snapshot").setNumber(1);
        } else {
            // Reset snapshot, ready for another one
//            NetworkTableInstance.getDefault().getTable("limelight").getEntry("snapshot").setNumber(0);
//            NetworkTableInstance.getDefault().getTable("limelight-bottom").getEntry("snapshot").setNumber(0);
        }
        for (int i = 0; i < nodes.length; i++) {
            boolean goRight = false;
            if (i == 0 && currentY < nodes[i]){
                goRight = true;
            } else if (i > 0){
                goRight = currentY < nodes[i] && currentY > (nodes[i] + nodes[i-1])/2 + nodeDeadZone/2;
            }
            boolean goLeft = false;
            if (i == nodes.length - 1 && currentY > nodes[i]){
                goLeft = true;
            } else if (i < nodes.length - 1) {
                goLeft = currentY > nodes[i] && currentY < (nodes[i] + nodes[i+1])/2 - nodeDeadZone/2;
            }

            if (goRight || goLeft) {
                return new Pose2d(node_x, nodes[i] , Rotation2d.fromDegrees(angle));
            }
        }
        return currentPose;
    }

    public static Pose2d getTargetPoseKroket(Pose2d currentPose, ArmManager.PositionState armState){
        double currentY = currentPose.getY();
        double currentX = currentPose.getX();
        double humanPlayerMiddleY = (HUMAN_PLAYER_Y[0] + HUMAN_PLAYER_Y[1])/2;
        boolean isBlueAlliance = DriverStation.getAlliance() == DriverStation.Alliance.Blue;

        double xOffset = 0;
        if (armState == ArmManager.PositionState.SPINDEXER_RETRACTED && isBlueAlliance){
            xOffset = -Constants.Drivetrain.AutoAlign.ARM_IN_X_OFFSET*3;
        } else if (armState == ArmManager.PositionState.SPINDEXER_RETRACTED){
            xOffset = Constants.Drivetrain.AutoAlign.ARM_IN_X_OFFSET*3;
        }
        if (armState != ArmManager.PositionState.SPINDEXER_RETRACTED) {
            // Make snapshot if aligning and using arm
//            NetworkTableInstance.getDefault().getTable("limelight").getEntry("snapshot").setNumber(1);
//            NetworkTableInstance.getDefault().getTable("limelight-bottom").getEntry("snapshot").setNumber(1);
        } else {
            // Reset snapshot, ready for another one
//            NetworkTableInstance.getDefault().getTable("limelight").getEntry("snapshot").setNumber(0);
//            NetworkTableInstance.getDefault().getTable("limelight-bottom").getEntry("snapshot").setNumber(0);
        }
        // Do not auto align if the robot is out of bounds
        if (isBlueAlliance) {
            if (currentY < humanPlayerMiddleY) {
                return new Pose2d(HUMAND_PLAYER_X_BLUE + xOffset, HUMAN_PLAYER_Y[0], Rotation2d.fromDegrees(180));
            } else {
                return new Pose2d(HUMAND_PLAYER_X_BLUE + xOffset, HUMAN_PLAYER_Y[1], Rotation2d.fromDegrees(180));
            }
        } else {
            if (currentY < humanPlayerMiddleY) {
                return new Pose2d(HUMAN_PLAYER_X_RED + xOffset, HUMAN_PLAYER_Y[0], new Rotation2d(0));
            } else {
                return new Pose2d(HUMAN_PLAYER_X_RED + xOffset, HUMAN_PLAYER_Y[1], new Rotation2d(0));
            }
        }
    }

    public static double getMaxVelocity(Pose2d currentPose){
        double targetVelocity;
        boolean isBlueAlliance = DriverStation.getAlliance() == DriverStation.Alliance.Blue;
        double currentX = currentPose.getX();
        boolean closeToGrid = (isBlueAlliance && (currentX < NODE_X_BLUE + Constants.Drivetrain.AutoAlign.ARM_IN_X_OFFSET))
                || (!isBlueAlliance && (currentX > NODE_X_RED - Constants.Drivetrain.AutoAlign.ARM_IN_X_OFFSET));
        if (!closeToGrid){
            return Constants.Drivetrain.AutoAlign.MAX_VELOCITY;
        } else {
            return Constants.Drivetrain.AutoAlign.MAX_VELOCITY / 2.5;
        }
    }
}
