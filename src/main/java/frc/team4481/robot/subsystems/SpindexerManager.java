//package frc.team4481.robot.subsystems;
//
//import frc.team4481.lib.subsystems.SubsystemManagerBase;
//
//public class SpindexerManager extends SubsystemManagerBase {
//
//    public enum ControlState {
//        DISABLED,
//        ENABLED,
//        ENABLED_NEW,
//        ENABLED_NEW_2,
//        DONE,
//        MANUAL,
//        POSITION,
//        REVERSE_FEEDER
//    }
//    private ControlState currentControlState = ControlState.DISABLED;
//
//    public enum DirectionState {
//        IDLE(0),
//        CLOCKWISE(1),
//        BOOST(-1.3),
//        COUNTERCLOCKWISE(-1),
//        COUNTERCLOCKWISESLOW(-0.4),
//        SPEED(0);
//
//        public final double value;
//        DirectionState(double value) {
//            this.value = value;
//        }
//    }
//    private DirectionState currentSpindexerState = DirectionState.IDLE;
//    private DirectionState currentFeederState = DirectionState.IDLE;
//
//
//    private boolean isOccupied = false;
//    private boolean coneInCutout = false;
//    private double spinSpeed = 0;
//
//    public void setControlState(ControlState pControlState) {
//        currentControlState = pControlState;
//    }
//    public ControlState getControlState() {
//        return currentControlState;
//    }
//
//    /**
//     * Manually set the direction of the spindexer
//     * @param state
//     */
//    public void setSpindexerState(DirectionState state) {
//        currentSpindexerState = state;
//    }
//    /**
//     * Get the direction of the spindexer
//     */
//    public DirectionState getSpindexerState() {
//        return currentSpindexerState;
//    }
//
//    /**
//     * Manually set the direction of the feeder wheels
//     * @param state
//     */
//    public void setFeederState(DirectionState state){
//        currentFeederState = state;
//    }
//    /**
//     * Get the direction of the feeder wheels
//     */
//    public DirectionState getFeederState() {
//        return currentFeederState;
//    }
//
//    /**
//     * Check if spindexer is occupied
//     */
//    public boolean getIsOccupied() {
//        return isOccupied;
//    }
//    /**
//     * Set occupation of the spindexer, (currently only allowed by sensor)
//     */
//    public void setIsOccupied(boolean occupied) {
//        isOccupied = occupied;
//    }
//
//    /**
//     * Check if cone is in cutout
//     */
//    public boolean getConeInCutout() {
//        return coneInCutout;
//    }
//    /**
//     * Set cone is in cutout, (currently only allowed by sensor)
//     */
//    public void setConeInCutout(boolean coneInCutout) {
//        this.coneInCutout = coneInCutout;
//    }
//
//    public double getSpinSpeed() {
//        return spinSpeed;
//    }
//
//    public void setSpinSpeed(double spinSpeed) {
//        this.spinSpeed = spinSpeed;
//    }
//}
