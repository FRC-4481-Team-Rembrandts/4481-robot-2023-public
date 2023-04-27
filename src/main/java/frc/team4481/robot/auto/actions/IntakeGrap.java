package frc.team4481.robot.auto.actions;

import edu.wpi.first.wpilibj.DataLogManager;
import frc.team4481.lib.auto.actions.Action;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.lib.util.CountingDelay;
import frc.team4481.robot.subsystems.*;
import static frc.team4481.robot.subsystems.ArmManager.PositionState.*;

public class IntakeGrap implements Action {
    SubsystemHandler mSubsystemManager = SubsystemHandler.getInstance();
    Gripper mGripper;
    GripperManager mGripperManager;
    Arm mArm;
    ArmManager mArmManager;
    Intake mIntake;
    IntakeManager mIntakeManager;
    CountingDelay stopDelay;
    double timeout;
    public IntakeGrap(double timeOut) {

        mGripper = (Gripper) mSubsystemManager.getSubsystemByClass(Gripper.class);
        mGripperManager = mGripper.getSubsystemManager();
        mArm = (Arm) mSubsystemManager.getSubsystemByClass(Arm.class);
        mArmManager = mArm.getSubsystemManager();
        mIntake = (Intake) mSubsystemManager.getSubsystemByClass(Intake.class);
        mIntakeManager = mIntake.getSubsystemManager();
        stopDelay = new CountingDelay();
        this.timeout = timeOut;
    }

    @Override
    public void start() {
        mArmManager.setCurrentPositionState(CUBE_INTAKE);
        mIntakeManager.setControlState(IntakeManager.ControlState.INTAKE);
        mGripperManager.setControlState(GripperManager.ControlState.INTAKE);
        stopDelay.reset();
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return mGripperManager.getDetectObject()
                || stopDelay.delay(timeout);
    }

    @Override
    public void done() {
        DataLogManager.log("Intake grap action done");
    }
}
