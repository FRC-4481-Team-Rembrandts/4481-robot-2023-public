package frc.team4481.robot.subsystems;

import frc.team4481.lib.subsystems.SubsystemManagerBase;


public class GripperManager extends SubsystemManagerBase {
    private boolean gripperOpen;
    public boolean getGripperOpen(){
        return gripperOpen;
    }
    public void setGripperOpen(boolean newGripperOpen){
        this.gripperOpen = newGripperOpen;
    }

    private int gripperCurrentLimit;
    public int getGripperCurrentLimit() {
        return gripperCurrentLimit;
    }
    public void setGripperCurrentLimit(int newGripperCurrentLimit) {this.gripperCurrentLimit = newGripperCurrentLimit;}

    private boolean detectObject;
    public boolean getDetectObject(){
        return detectObject;
    }
    public void setDetectObject(boolean newDetectObject){
        this.detectObject = newDetectObject;
    }

    private boolean sensorDisconnected = false;
    public boolean getSensorDisconnected(){
        return sensorDisconnected;
    }
    public void setSensorDisconnected(boolean newSensorDisconnected){this.sensorDisconnected = newSensorDisconnected;}

    private double gripperSpeed;
    public double getGripperSpeed() {
        return gripperSpeed;
    }
    public void setGripperSpeed(double gripperSpeed) {
        this.gripperSpeed = gripperSpeed;
    }

    private ControlState currentControlState = ControlState.DISABLED;

    public enum ControlState {
        DISABLED,
        HOLD,
        SHOOT,
        INTAKE,
        OUTTAKE
    }

    public void setControlState(ControlState pControlState) {
        currentControlState = pControlState;
    }

    public ControlState getControlState() {
        return currentControlState;
    }


}
