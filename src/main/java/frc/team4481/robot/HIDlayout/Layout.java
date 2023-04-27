package frc.team4481.robot.HIDlayout;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4481.lib.controller.IPS4HID;
import frc.team4481.lib.hid.HIDLayout;
import frc.team4481.lib.controller.ControlDevice;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.lib.throwable.HardwareException;
import frc.team4481.robot.subsystems.*;
import frc.team4481.robot.util.GamePieceHandler;

import static frc.team4481.lib.controller.IPS4HID.Axis.TRIGGER_L2;
import static frc.team4481.lib.controller.IPS4HID.Axis.TRIGGER_R2;
import static frc.team4481.lib.controller.IPS4HID.Button.*;
import static frc.team4481.lib.controller.IPS4HID.DpadButton.DPAD_E;
import static frc.team4481.lib.controller.IPS4HID.DpadButton.DPAD_W;
import static frc.team4481.robot.util.GamePieceHandler.GamePiece.CONE;
import static frc.team4481.robot.util.GamePieceHandler.GamePiece.CUBE;

public class Layout extends HIDLayout {
    private final SubsystemHandler subsystemHandler = SubsystemHandler.getInstance();

    // TODO add required subsystems

    private Intake intake;
    private IntakeManager intakeManager;
//    private Spindexer spindexer;
//    private SpindexerManager spindexerManager;
    private Gripper gripper;
    private GripperManager gripperManager;
    private final GamePieceHandler gamePieceHandler = GamePieceHandler.getInstance();

    public Layout(ControlDevice driver, ControlDevice operator) {
        super(driver, operator);
    }


    @Override
    public void getSubsystemManagers() {
        //TODO add required subsystems
//        intake = (Intake) subsystemHandler.getSubsystemByClass(Intake.class);
//        intakeManager = intake.getSubsystemManager();
//        spindexer = (Spindexer) subsystemHandler.getSubsystemByClass(Spindexer.class);
//        spindexerManager = spindexer.getSubsystemManager();
        gripper = (Gripper) subsystemHandler.getSubsystemByClass(Gripper.class);
        gripperManager = gripper.getSubsystemManager();
    }

    @Override
    public void updateOrange() throws HardwareException {
//        intakeManager.setControlState(IntakeManager.controlState.TUNING);

//        if (driver.getButtonValue(BUMPER_R1)) {
//            intakeManager.setControlState(IntakeManager.controlState.INTAKE);
//        } else if (driver.getAxisValue(TRIGGER_R2)>0.1) {
//            intakeManager.setControlState(IntakeManager.controlState.REVERSE);
//        } else{
//            intakeManager.setControlState(IntakeManager.controlState.DISABLED);
//        }
//        if (driver.getRawButtonPressed(SHARE.id)) {
//            if (spindexerManager.getControlState() == SpindexerManager.ControlState.ENABLED) {
//                spindexerManager.setControlState(SpindexerManager.ControlState.DISABLED);
//            } else {
//                spindexerManager.setControlState(SpindexerManager.ControlState.ENABLED);
//            }
//        }
//
//        if (driver.getButtonValue(OPTIONS)) {
//            spindexerManager.setControlState(SpindexerManager.ControlState.MANUAL);
//            spindexerManager.setFeederState(SpindexerManager.DirectionState.COUNTERCLOCKWISE);
//        } else if (driver.getDpadValue(DPAD_E)) {
//            spindexerManager.setControlState(SpindexerManager.ControlState.MANUAL);
//            spindexerManager.setSpindexerState(SpindexerManager.DirectionState.CLOCKWISE);
//        } else if (driver.getDpadValue(DPAD_W)) {
//            spindexerManager.setControlState(SpindexerManager.ControlState.MANUAL);
//            spindexerManager.setSpindexerState(SpindexerManager.DirectionState.COUNTERCLOCKWISE);
//        } else if (driver.getButtonValue(LEFTSTICK_BUTTON)) {
//            spindexerManager.setControlState(SpindexerManager.ControlState.DISABLED);
//        } else if (driver.getAxisValue(TRIGGER_R2)>0.1) {
//            // Reverse feeder with intake reverse
//            spindexerManager.setControlState(SpindexerManager.ControlState.REVERSE_FEEDER);
//        } else if (spindexerManager.getControlState() == SpindexerManager.ControlState.MANUAL) {
//            // Disable spindexer automatically if it is not enabled
//            spindexerManager.setControlState(SpindexerManager.ControlState.DISABLED);
//        } else if (driver.getButtonValue(RIGHTSTICK_BUTTON)) {
//            spindexerManager.setControlState(SpindexerManager.ControlState.POSITION);
//        }

        // Gripper manual controls
        if (driver.getAxisValue(TRIGGER_R2) > 0.1) {
            // Bind right trigger to gripper outtake
            gripperManager.setControlState(GripperManager.ControlState.OUTTAKE);
        } else if (driver.getAxisValue(TRIGGER_L2) > 0.1){
            // Bind left trigger to gripper intake
            gripperManager.setControlState(GripperManager.ControlState.INTAKE);
        } else if (driver.getButtonValue(BUMPER_R1)) {
            // Bind right bumper to gripper shooting
            gripperManager.setControlState(GripperManager.ControlState.SHOOT);
        }

        if (driver.getRawButtonPressed(BUMPER_L1.id)) {
            if (gamePieceHandler.getGamePiece() == CONE) {
                gamePieceHandler.setGamePiece(CUBE);
            } else {
                gamePieceHandler.setGamePiece(CONE);
            }
        }
        SmartDashboard.putString("Game Piece", gamePieceHandler.getGamePiece().toString());
    }


    @Override
    public void updateBlack() throws HardwareException {

    }
}
