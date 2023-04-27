package frc.team4481.robot.HIDlayout;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4481.lib.controller.ControlDevice;
import frc.team4481.lib.hid.HIDLayout;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.lib.throwable.HardwareException;
import frc.team4481.robot.subsystems.*;
import frc.team4481.robot.util.GamePieceHandler;

import static frc.team4481.lib.controller.IPS4HID.Axis.TRIGGER_L2;
import static frc.team4481.lib.controller.IPS4HID.Axis.TRIGGER_R2;
import static frc.team4481.lib.controller.IPS4HID.Button.*;
import static frc.team4481.robot.util.GamePieceHandler.GamePiece.CONE;
import static frc.team4481.robot.util.GamePieceHandler.GamePiece.CUBE;

public class TuneLayout extends HIDLayout {
    private final SubsystemHandler subsystemHandler = SubsystemHandler.getInstance();
    private final GamePieceHandler gamePieceHandler = GamePieceHandler.getInstance();

    private Arm arm;
    private ArmManager armManager;
    private Intake intake;
    private IntakeManager intakeManager;
    private Gripper gripper;
    private GripperManager gripperManager;

    public TuneLayout(ControlDevice driver, ControlDevice operator) {
        super(driver, operator);
    }


    @Override
    public void getSubsystemManagers() {
        arm = (Arm) subsystemHandler.getSubsystemByClass(Arm.class);
        armManager = arm.getSubsystemManager();
        intake = (Intake) subsystemHandler.getSubsystemByClass(Intake.class);
        intakeManager = intake.getSubsystemManager();
        gripper = (Gripper) subsystemHandler.getSubsystemByClass(Gripper.class);
        gripperManager = gripper.getSubsystemManager();
    }

    @Override
    public void updateOrange() throws HardwareException {



    }


    @Override
    public void updateBlack() throws HardwareException {
        if (armManager.getCurrentControlState() != ArmManager.ControlState.CALIBRATING) {
            armManager.setCurrentControlState(ArmManager.ControlState.MANUAL);
        }
//        intakeManager.setControlState(IntakeManager.controlState.TUNING);

        if (operator.getAxisValue(TRIGGER_R2) > 0.1) {
            // Bind right trigger to gripper outtake
            gripperManager.setControlState(GripperManager.ControlState.OUTTAKE);
        } else if (operator.getAxisValue(TRIGGER_L2) > 0.1){
            // Bind left trigger to gripper intake
            gripperManager.setControlState(GripperManager.ControlState.INTAKE);
        }


        if (operator.getRawButtonPressed(BUMPER_R1.id)) {
            if (armManager.getCurrentTuneState() == ArmManager.TuneState.TELESCOPE) {
                armManager.setCurrentTuneState(ArmManager.TuneState.PIVOT);
            } else {
                armManager.setCurrentTuneState(ArmManager.TuneState.TELESCOPE);
            }
        }

        if (operator.getButtonValue(TRIANGLE) )
        {
            armManager.setCurrentPositionState(ArmManager.PositionState.HIGH);
        }

        //move pivot to middle position
//        if (operator.getButtonValue(CIRCLE) )
//        {
//            armManager.setCurrentPositionState(ArmManager.PositionState.PLAYER_STATION);
//        }

        //move pivot to low position
        if (operator.getButtonValue(CROSS) )
        {
            armManager.setCurrentPositionState(ArmManager.PositionState.SPINDEXER_RETRACTED);
        }

//        if (operator.getButtonValue(SQUARE) )
//        {
//            armManager.setCurrentPositionState(ArmManager.PositionState.SPINDEXER_RETRACTED);
//        }

        if (operator.getRawButtonPressed(BUMPER_L1.id)) {
            if (gamePieceHandler.getGamePiece() == CONE) {
                gamePieceHandler.setGamePiece(CUBE);
            } else {
                gamePieceHandler.setGamePiece(CONE);
            }
        }
        SmartDashboard.putString("Game Piece", gamePieceHandler.getGamePiece().toString());

    }
}
