package frc.team4481.robot.auto.actions;

import edu.wpi.first.wpilibj.DataLogManager;
import frc.team4481.lib.auto.actions.Action;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.lib.util.CountingDelay;
import frc.team4481.robot.subsystems.*;

public class DexerPickupAction implements Action {
    SubsystemHandler mSubsystemManager = SubsystemHandler.getInstance();
    Gripper mGripper;
    GripperManager mGripperManager;
    Arm mArm;
    ArmManager mArmManager;
    Intake mIntake;
    IntakeManager mIntakeManager;
    CountingDelay stopDelay;
    double timeout;
    public DexerPickupAction(double timeOut) {

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
        mArmManager.setCurrentPositionState(ArmManager.PositionState.SPINDEXER_RETRACTED);
        stopDelay.reset();
    }

    @Override
    public void update() {
            if (mIntakeManager.getIsOccupied()){
                mGripperManager.setControlState(GripperManager.ControlState.INTAKE);
                mArmManager.setCurrentPositionState(ArmManager.PositionState.SPINDEXER_EXTENDED);
            }
    }

    @Override
    public boolean isFinished() {
        return mGripperManager.getDetectObject()
                || stopDelay.delay(timeout);
    }

    @Override
    public void done() {
        DataLogManager.log("Dexer grap action done");
        mArmManager.setCurrentPositionState(ArmManager.PositionState.SPINDEXER_RETRACTED);
    }
}
