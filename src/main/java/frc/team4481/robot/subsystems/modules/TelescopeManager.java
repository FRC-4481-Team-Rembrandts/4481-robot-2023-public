package frc.team4481.robot.subsystems.modules;

import frc.team4481.lib.subsystems.SubsystemManagerBase;
import static frc.team4481.robot.Constants.Telescope.Comp.*;
import static frc.team4481.robot.Constants.Telescope.Comp.Positions.*;

public class TelescopeManager extends SubsystemManagerBase {

    /**
     * Whether the submodule should be allowed to move
     */
    private boolean movable = false;

    private ControlState currentControlState;
    private PositionState currentPositionState;
    private MovementState currentMovementState;
    private ExtensionState currentExtensionState = ExtensionState.RETRACTED;
    private boolean isCalibrated = false;

    /**
     * State that the module is in.
     * This can either be disabled, calibrating, automatic or manual.
     * In automatic, the pivot will move between preset positions.
     * In manual, the arm can be moved to any position
     */
    public enum ControlState {
        DISABLED,
        CALIBRATING,
        AUTOMATIC,
        MANUAL
    }

    /**
     * Whether the telescope is extended or retracted.
     * The specific distance that the telescope needs to extend
     * is governed by the position state.
     * The position state is linked to the extension state using the pivot
     * position in {@code ArmManager}
     */
    public enum ExtensionState {
        RETRACTED,
        EXTENDED
    }


    /**
     * Preset positions of the telescope.
     * Positions contain the extension in centimeters corresponding
     * to that position.
     * Extension in centimeters can be accessed using {@code getValue()}.
     */
    public enum PositionState {
        RETRACTED(MIN_EXTEND),
        CUBE_HIGH(CUBE_HIGH_POS),
        CUBE_MIDDLE(MIN_EXTEND),
        CUBE_LOW(MIN_EXTEND),
        CONE_HIGH(CONE_HIGH_POS),
        CONE_MIDDLE(CONE_MID_POS),
        CONE_LOW(MIN_EXTEND),
        PLAYER_STATION(MIN_EXTEND),
        CONE_DEXER(CONE_DEXER_POS),
        CUBE_DEXER(CUBE_DEXER_POS),
        CONE_GROUND(CONE_GROUND_POS),
        CUBE_GROUND(CUBE_GROUND_POS),
        CONE_SMASH(CONE_SMASH_POS),
        CUBE_INTAKE(CUBE_INTAKE_POS);

        private final double extension;

        PositionState(double extension){
            this.extension = extension;
        }

        public double getValue(){
            return this.extension;
        }
    }

    public enum MovementState {
        ON_TARGET,
        MOVING
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

    public ExtensionState getCurrentExtensionState() {
        return currentExtensionState;
    }

    public void setCurrentExtensionState(ExtensionState currentExtensionState) {
        this.currentExtensionState = currentExtensionState;
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

    public boolean isCalibrated(){
        return isCalibrated;
    }

    public void setCalibrated(boolean calibrated){
        isCalibrated = calibrated;
    }
}
