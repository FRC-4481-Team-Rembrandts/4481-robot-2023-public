package frc.team4481.robot.subsystems.modules;

import frc.team4481.lib.subsystems.SubsystemManagerBase;

import static frc.team4481.robot.Constants.Pivot.Comp.Positions.*;

public class PivotManager  extends SubsystemManagerBase {

    private boolean armInRobot;
    public boolean getArmInRobot(){
        return armInRobot;
    }
    public void setArmInRobot(boolean newArmInRobot){
        this.armInRobot = newArmInRobot;
    }

    private ControlState currentControlState = ControlState.DISABLED;

    /**
     * Whether the submodule should be allowed to move
     */
    private boolean movable = false;

    /**
     * Whether the pivot is in a zone where it can safely extend without hitting the grid
     */
    private boolean inSafeZone = false;

    MovementState currentMovementState;
    PositionState currentPositionState;


    /**
     * State that the module is in.
     * This can either be disabled, automatic or manual.
     * In automatic, the pivot will move between preset positions.
     * In manual, the arm can be moved to any position.
     */
    public enum ControlState {
        DISABLED,
        AUTOMATIC,
        MANUAL,
    }

    /**
     * State of the movement of the pivot.
     * It can either be moving to a position (MOVING),
     * on a target position (ON_TARGET),
     * or waiting for the telescope to retract (WAITING).
     */
    public enum MovementState {
        MOVING,
        ON_TARGET
    }

    /**
     * Preset positions of the pivot.
     * Positions contain the angle in degrees corresponding
     * to that position.
     * Angle in degrees can be accessed using {@code getValue()}.
     */
    public enum PositionState {
        IDLE(IDLE_POS),
        CUBE_HIGH(CUBE_HIGH_POS),
        CUBE_MIDDLE(CUBE_MID_POS),
        CUBE_LOW(CUBE_LOW_POS),
        CONE_HIGH(CONE_HIGH_POS),
        CONE_MIDDLE(CONE_MID_POS),
        CONE_LOW(CONE_LOW_POS),
        CONE_PLAYER_STATION(CONE_PLAYER_POS),
        CUBE_PLAYER_STATION(CUBE_PLAYER_POS),
        CONE_DEXER(CONE_DEXER_POS),
        CUBE_DEXER(CUBE_DEXER_POS),
        CONE_GROUND(CONE_GROUND_POS),
        CUBE_GROUND(CUBE_GROUND_POS),
        CONE_SMASH(CONE_DEXER_POS),
        CUBE_INTAKE(CUBE_INTAKE_POS);


        private final double angle;

        PositionState(double angle){
            this.angle = angle;
        }

        public double getValue(){
            return this.angle;
        }
    }



    public void setControlState(ControlState pControlState) {
        currentControlState = pControlState;
    }

    public ControlState getControlState() {
        return currentControlState;
    }


    /**
     * Whether the submodule should be allowed to move
     *
     * @return Whether the submodule should be allowed to move
     */
    public boolean isMovable() {
        return movable;
    }


    /**
     * Sets if the submodule should be allowed to move
     *
     * @param movable if the submodule should be allowed to move
     */
    public void setMovable(boolean movable) {
        this.movable = movable;
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

    public MovementState getCurrentMovementState() {
        return currentMovementState;
    }

    public void setCurrentMovementState(MovementState currentMovementState) {
        this.currentMovementState = currentMovementState;
    }

    public boolean isInSafeZone() {
        return inSafeZone;
    }

    public void setInSafeZone(boolean inSafeZone) {
        this.inSafeZone = inSafeZone;
    }


}

