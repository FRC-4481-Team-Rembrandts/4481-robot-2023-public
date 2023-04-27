package frc.team4481.robot.subsystems;

import frc.team4481.lib.subsystems.SubsystemManagerBase;

public class ArmManager extends SubsystemManagerBase {
    private ControlState currentControlState = ControlState.DISABLED;
    private PositionState currentPositionState = PositionState.SPINDEXER_RETRACTED;
    private boolean onTarget = false;

    private boolean armInRobot;
    public boolean getArmInRobot(){
        return armInRobot;
    }
    public void setArmInRobot(boolean newArmInRobot){
        this.armInRobot = newArmInRobot;
    }

    /**
     * Whether both the telescope and the pivot have updated the movement state.
     */
    private boolean updated = false;

    public enum ControlState {
        DISABLED,
        CALIBRATING,
        AUTOMATIC,
        MANUAL
    }

    /**
     * Position of the arm.
     * This is a combination of pivot and telescope position.
     */
    public enum PositionState {
        HIGH,
        MIDDLE,
        LOW,
        PLAYER_STATION,
        SPINDEXER_RETRACTED,
        SPINDEXER_EXTENDED,
        GROUND,
        CONE_SMASH,
        CUBE_INTAKE
    }

    public enum TuneState {
        PIVOT,
        TELESCOPE
    }

    private TuneState currentTuneState = TuneState.TELESCOPE;

    public TuneState getCurrentTuneState() {
        return currentTuneState;
    }

    public void setCurrentTuneState(TuneState currentTuneState) {
        this.currentTuneState = currentTuneState;
    }

    public ControlState getCurrentControlState() {
        return currentControlState;
    }

    public void setCurrentControlState(ControlState currentControlState) {
        this.currentControlState = currentControlState;
    }

    public PositionState getCurrentPositionState() {
        return currentPositionState;
    }

    public void setCurrentPositionState(PositionState currentPositionState) {
        this.currentPositionState = currentPositionState;
    }

    public boolean isOnTarget() {
        return onTarget;
    }

    public void setOnTarget(boolean onTarget) {
        this.onTarget = onTarget;
    }

    public boolean isUpdated() {
        return updated;
    }

    public void setUpdated(boolean updated) {
        this.updated = updated;
    }
}
