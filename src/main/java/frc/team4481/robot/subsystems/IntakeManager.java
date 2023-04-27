package frc.team4481.robot.subsystems;

import frc.team4481.lib.subsystems.SubsystemManagerBase;

public class IntakeManager extends SubsystemManagerBase {
    private boolean isOccupied = false;
    /**
     * Check if dexer is occupied
     */
    public boolean getIsOccupied() {
        return isOccupied;
    }
    /**
     * Set occupation of the dexer, (currently only allowed by sensor)
     */
    public void setIsOccupied(boolean occupied) {
        isOccupied = occupied;
    }
    private ControlState currentControlState = ControlState.DISABLED;

    public enum ControlState {
        DISABLED,
        INTAKE,
        REVERSE,
        ROLLING,
        TUNING
    }

    public void setControlState(ControlState pControlState) {
        currentControlState = pControlState;
    }

    public ControlState getControlState() {
        return currentControlState;
    }
}
