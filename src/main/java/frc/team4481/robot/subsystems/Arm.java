package frc.team4481.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.team4481.lib.subsystems.SubsystemBase;
import frc.team4481.robot.subsystems.modules.Pivot;
import frc.team4481.robot.subsystems.modules.PivotManager;
import frc.team4481.robot.subsystems.modules.Telescope;
import frc.team4481.robot.subsystems.modules.TelescopeManager;
import frc.team4481.robot.util.GamePieceHandler;

import java.util.Arrays;
import java.util.List;

import static frc.team4481.robot.util.GamePieceHandler.GamePiece.*;

public class Arm extends SubsystemBase<ArmManager> {
    private List<SubsystemBase> armModules;
    private Telescope telescope;
    private Pivot pivot;

    private TelescopeManager telescopeManager;
    private PivotManager pivotManager;

    private final GamePieceHandler gamePieceHandler = GamePieceHandler.getInstance();

    private Mechanism2d arm_mech;
    private MechanismLigament2d arm_mech_pivot_curr;
    private MechanismLigament2d arm_mech_pivot_des;
    private MechanismLigament2d arm_mech_telescope_curr;
    private MechanismLigament2d arm_mech_telescope_des;

    public Arm() {
        name = "Arm";
        subsystemManager = new ArmManager();

        telescope = new Telescope();
        pivot = new Pivot();

        telescopeManager = telescope.getSubsystemManager();
        pivotManager = pivot.getSubsystemManager();

        armModules = Arrays.asList(telescope, pivot);

        // the main mechanism object
        arm_mech = new Mechanism2d(4, 3);
        // the mechanism root node
        MechanismRoot2d arm_mech_root = arm_mech.getRoot("arm", 2, 0);

        // Superstructure node
        MechanismLigament2d arm_mech_superstructure = arm_mech_root.append(
                        new MechanismLigament2d("superstructure", 1.2, 90, 10, new Color8Bit(Color.kAqua)));
        // Pivot node
        arm_mech_pivot_curr = arm_mech_superstructure.append(
                        new MechanismLigament2d("pivot_curr", 0.5, 90, 8, new Color8Bit(Color.kAquamarine)));

        arm_mech_pivot_des = arm_mech_superstructure.append(
                new MechanismLigament2d("pivot_des", 0.5, 90, 6, new Color8Bit(Color.kDeepPink)));
        // Telescope node
        arm_mech_telescope_curr = arm_mech_pivot_curr.append(
                        new MechanismLigament2d("telescope_curr", 0, 0, 6, new Color8Bit(Color.kAzure)));

        arm_mech_telescope_des = arm_mech_pivot_des.append(
                new MechanismLigament2d("telescope_des", 0, 0, 4, new Color8Bit(Color.kHotPink)));
    }

    @Override
    public void onStart(double timestamp) {
        subsystemManager.setCurrentControlState(ArmManager.ControlState.CALIBRATING);
//        telescopeManager.setCalibrated(false);
        subsystemManager.setCurrentPositionState(ArmManager.PositionState.SPINDEXER_RETRACTED);

        armModules.forEach(s -> s.onStart(timestamp));
    }

    @Override
    public void onLoop(double timestamp) {
        // Test mobility states
        checkTelescopeMovable();
        checkPivotMovable();

        // Set states of submodules
        setTelescopePosition();
        setPivotPosition();
        //The positions of the modules have now been updated
        subsystemManager.setUpdated(true);

        switch (subsystemManager.getCurrentControlState()) {
            case DISABLED:
                telescopeManager.setCurrentControlState(TelescopeManager.ControlState.DISABLED);
                pivotManager.setCurrentControlState(PivotManager.ControlState.DISABLED);
                break;
            case CALIBRATING:
                // First check if calibration is finished. If so, move to AUTOMATIC instead
                if(telescopeManager.isCalibrated()) {
                    subsystemManager.setCurrentControlState(ArmManager.ControlState.AUTOMATIC);
                    break;
                }

                telescopeManager.setCurrentControlState(TelescopeManager.ControlState.CALIBRATING);
                pivotManager.setCurrentControlState(PivotManager.ControlState.AUTOMATIC);
                break;
            case AUTOMATIC:
                telescopeManager.setCurrentControlState(TelescopeManager.ControlState.AUTOMATIC);
                pivotManager.setControlState(PivotManager.ControlState.AUTOMATIC);
                break;
            case MANUAL:
                if (subsystemManager.getCurrentTuneState() == ArmManager.TuneState.TELESCOPE) {
                    telescopeManager.setCurrentControlState(TelescopeManager.ControlState.MANUAL);
                    pivotManager.setCurrentControlState(PivotManager.ControlState.DISABLED);
                } else {
                    telescopeManager.setCurrentControlState(TelescopeManager.ControlState.MANUAL);
                    // TODO remove cursed if statement
                    if (gamePieceHandler.getGamePiece() == CONE) {
                        telescopeManager.setCurrentPositionState(TelescopeManager.PositionState.RETRACTED);
                    } else {
                        telescopeManager.setCurrentPositionState(TelescopeManager.PositionState.CONE_HIGH);
                    }
                    pivotManager.setCurrentControlState(PivotManager.ControlState.MANUAL);
                }
                break;
        }

        armModules.forEach(s -> s.onLoop(timestamp));
        //update value from PivotManager to ArmManager
        subsystemManager.setArmInRobot(pivotManager.getArmInRobot());

        // Set on target position
        boolean onTarget = telescopeManager.getCurrentExtensionState() == TelescopeManager.ExtensionState.EXTENDED
                && telescopeManager.getCurrentMovementState() == TelescopeManager.MovementState.ON_TARGET
                && pivotManager.getCurrentMovementState() == PivotManager.MovementState.ON_TARGET
                && subsystemManager.isUpdated();

        subsystemManager.setOnTarget(onTarget);
    }

    @Override
    public void onStop(double timestamp) {
        terminate();

        armModules.forEach(s -> s.onStop(timestamp));
    }

    @Override
    public void readPeriodicInputs() {
        armModules.forEach(SubsystemBase::readPeriodicInputs);

        arm_mech_pivot_curr.setAngle(pivot.getAbsoluteAngle());
        arm_mech_telescope_curr.setLength(telescope.getPositionCurrent()/100);

        arm_mech_pivot_des.setAngle(pivot.getAbsoluteAngleTarget());
        arm_mech_telescope_des.setLength(telescope.getPositionDesired()/100);
    }

    @Override
    public void writePeriodicOutputs() {
        armModules.forEach(SubsystemBase::writePeriodicOutputs);

        SmartDashboard.putData("ArmMech2d", arm_mech);
    }

    @Override
    public void zeroSensors() {
        armModules.forEach(SubsystemBase::zeroSensors);
    }

    @Override
    public void terminate() {
        subsystemManager.setCurrentControlState(ArmManager.ControlState.DISABLED);

        armModules.forEach(SubsystemBase::terminate);
    }

    @Override
    public void outputData() {
        SmartDashboard.putString("arm/control state", subsystemManager.getCurrentControlState().toString());
        SmartDashboard.putString("arm/position state", subsystemManager.getCurrentPositionState().toString());
        SmartDashboard.putString("arm/tune state", subsystemManager.getCurrentTuneState().toString());
        SmartDashboard.putBoolean("arm/arm in robot", subsystemManager.getArmInRobot());

        armModules.forEach(SubsystemBase::outputData);
    }

    /**
     * Sets the {@code PositionState} of the {@code Telescope}
     */
    private void setTelescopePosition() {
        ArmManager.PositionState armPosition = subsystemManager.getCurrentPositionState();
        if (gamePieceHandler.getGamePiece() == CUBE) {
            switch (armPosition){
                case HIGH:
                    telescopeManager.setCurrentPositionState(TelescopeManager.PositionState.CUBE_HIGH);
                    break;
                case MIDDLE:
                    telescopeManager.setCurrentPositionState(TelescopeManager.PositionState.CUBE_MIDDLE);
                    break;
                case LOW:
                    telescopeManager.setCurrentPositionState(TelescopeManager.PositionState.CUBE_LOW);
                    break;
                case PLAYER_STATION:
                    telescopeManager.setCurrentPositionState(TelescopeManager.PositionState.PLAYER_STATION);
                    break;
                case SPINDEXER_EXTENDED:
                    telescopeManager.setCurrentPositionState(TelescopeManager.PositionState.CUBE_DEXER);
                    break;
                case SPINDEXER_RETRACTED:
                    telescopeManager.setCurrentPositionState(TelescopeManager.PositionState.RETRACTED);
                    break;
                case GROUND:
                    telescopeManager.setCurrentPositionState(TelescopeManager.PositionState.CUBE_GROUND);
                    break;
                case CONE_SMASH:
                    telescopeManager.setCurrentPositionState(TelescopeManager.PositionState.CONE_SMASH);
                    break;
                case CUBE_INTAKE:
                    telescopeManager.setCurrentPositionState(TelescopeManager.PositionState.CUBE_INTAKE);
                    break;
            }
        } else {
            switch (armPosition) {
                case HIGH:
                    telescopeManager.setCurrentPositionState(TelescopeManager.PositionState.CONE_HIGH);
                    break;
                case MIDDLE:
                    telescopeManager.setCurrentPositionState(TelescopeManager.PositionState.CONE_MIDDLE);
                    break;
                case LOW:
                    telescopeManager.setCurrentPositionState(TelescopeManager.PositionState.CONE_LOW);
                    break;
                case PLAYER_STATION:
                    telescopeManager.setCurrentPositionState(TelescopeManager.PositionState.PLAYER_STATION);
                    break;
                case SPINDEXER_EXTENDED:
                    telescopeManager.setCurrentPositionState(TelescopeManager.PositionState.CONE_DEXER);
                    break;
                case SPINDEXER_RETRACTED:
                    telescopeManager.setCurrentPositionState(TelescopeManager.PositionState.RETRACTED);
                    break;
                case GROUND:
                    telescopeManager.setCurrentPositionState(TelescopeManager.PositionState.CONE_GROUND);
                    break;
                case CONE_SMASH:
                    telescopeManager.setCurrentPositionState(TelescopeManager.PositionState.CONE_SMASH);
                    break;
            }
        }
    }

    /**
     * Sets the {@code PositionState} of the {@code Pivot}
     */
    private void setPivotPosition() {
        ArmManager.PositionState armPosition = subsystemManager.getCurrentPositionState();
        if (gamePieceHandler.getGamePiece() == CUBE){
            switch (armPosition){
                case HIGH:
                    pivotManager.setCurrentPositionState(PivotManager.PositionState.CUBE_HIGH);
                    break;
                case MIDDLE:
                    pivotManager.setCurrentPositionState(PivotManager.PositionState.CUBE_MIDDLE);
                    break;
                case LOW:
                    pivotManager.setCurrentPositionState(PivotManager.PositionState.CUBE_LOW);
                    break;
                case PLAYER_STATION:
                    pivotManager.setCurrentPositionState(PivotManager.PositionState.CUBE_PLAYER_STATION);
                    break;
                case SPINDEXER_EXTENDED:
                    pivotManager.setCurrentPositionState(PivotManager.PositionState.CUBE_DEXER);
                    break;
                case SPINDEXER_RETRACTED:
                    pivotManager.setCurrentPositionState(PivotManager.PositionState.IDLE);
                    break;
                case GROUND:
                    pivotManager.setCurrentPositionState(PivotManager.PositionState.CUBE_GROUND);
                    break;
                case CONE_SMASH:
                    pivotManager.setCurrentPositionState(PivotManager.PositionState.CONE_SMASH);
                case CUBE_INTAKE:
                    pivotManager.setCurrentPositionState(PivotManager.PositionState.CUBE_INTAKE);
                    break;
            }
        } else {
            switch (armPosition){
                case HIGH:
                    pivotManager.setCurrentPositionState(PivotManager.PositionState.CONE_HIGH);
                    break;
                case MIDDLE:
                    pivotManager.setCurrentPositionState(PivotManager.PositionState.CONE_MIDDLE);
                    break;
                case LOW:
                    pivotManager.setCurrentPositionState(PivotManager.PositionState.CONE_LOW);
                    break;
                case PLAYER_STATION:
                    pivotManager.setCurrentPositionState(PivotManager.PositionState.CONE_PLAYER_STATION);
                    break;
                case SPINDEXER_EXTENDED:
                    pivotManager.setCurrentPositionState(PivotManager.PositionState.CONE_DEXER);
                    break;
                case SPINDEXER_RETRACTED:
                    pivotManager.setCurrentPositionState(PivotManager.PositionState.IDLE);
                    break;
                case GROUND:
                    pivotManager.setCurrentPositionState(PivotManager.PositionState.CONE_GROUND);
                    break;
                case CONE_SMASH:
                    pivotManager.setCurrentPositionState(PivotManager.PositionState.CONE_SMASH);
                    break;
            }
        }
    }

    private void checkPivotMovable(){
        boolean pivotMovable =
                ((telescopeManager.getCurrentExtensionState() == TelescopeManager.ExtensionState.RETRACTED
                        && telescopeManager.getCurrentMovementState() == TelescopeManager.MovementState.ON_TARGET)
                        || (pivotManager.isInSafeZone()) )
                        && telescopeManager.getCurrentControlState() == TelescopeManager.ControlState.AUTOMATIC

                        || (DriverStation.isAutonomous() && telescopeManager.getCurrentControlState() == TelescopeManager.ControlState.CALIBRATING );

        pivotManager.setMovable(pivotMovable);
    }

    private void checkTelescopeMovable(){
        boolean telescopeMovable =
                pivotManager.getCurrentMovementState() == PivotManager.MovementState.ON_TARGET
                || (pivotManager.isInSafeZone());

        telescopeManager.setMovable(telescopeMovable);
    }
}
