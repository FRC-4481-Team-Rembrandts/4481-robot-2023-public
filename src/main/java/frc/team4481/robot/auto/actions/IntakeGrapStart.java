package frc.team4481.robot.auto.actions;

import edu.wpi.first.wpilibj.DataLogManager;
import frc.team4481.lib.auto.actions.Action;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.lib.util.CountingDelay;
import frc.team4481.robot.subsystems.*;

import static frc.team4481.robot.subsystems.ArmManager.PositionState.CUBE_INTAKE;

public class IntakeGrapStart implements Action {
    SubsystemHandler mSubsystemManager = SubsystemHandler.getInstance();
    Gripper mGripper;
    GripperManager mGripperManager;
    Arm mArm;
    ArmManager mArmManager;
    Intake mIntake;
    IntakeManager mIntakeManager;
    public IntakeGrapStart() {

        mGripper = (Gripper) mSubsystemManager.getSubsystemByClass(Gripper.class);
        mGripperManager = mGripper.getSubsystemManager();
        mArm = (Arm) mSubsystemManager.getSubsystemByClass(Arm.class);
        mArmManager = mArm.getSubsystemManager();
        mIntake = (Intake) mSubsystemManager.getSubsystemByClass(Intake.class);
        mIntakeManager = mIntake.getSubsystemManager();
    }

    @Override
    public void start() {
        mArmManager.setCurrentPositionState(CUBE_INTAKE);
        mIntakeManager.setControlState(IntakeManager.ControlState.INTAKE);
        mGripperManager.setControlState(GripperManager.ControlState.INTAKE);
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void done() {
        DataLogManager.log("Intake grap start action done");
    }
}
