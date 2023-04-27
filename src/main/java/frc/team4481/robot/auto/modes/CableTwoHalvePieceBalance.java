package frc.team4481.robot.auto.modes;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.team4481.lib.auto.actions.ParallelAction;
import frc.team4481.lib.auto.actions.SeriesAction;
import frc.team4481.lib.auto.actions.WaitAction;
import frc.team4481.lib.auto.mode.AutoModeBase;
import frc.team4481.lib.auto.mode.AutoModeEndedException;
import frc.team4481.robot.Constants;
import frc.team4481.robot.auto.actions.*;
import frc.team4481.robot.auto.selector.AutoMode;
import frc.team4481.robot.subsystems.ArmManager;
import frc.team4481.robot.subsystems.IntakeManager;
import frc.team4481.robot.util.GamePieceHandler;

/**
 * Scores initial cone
 * collects cube closest to cable duct
 * scores high closest to cable duct
 * collects next cube
 * scores mid closest to cable duct
 */
@AutoMode(displayName = "[Cable] 2.5 Piece Balance")
public class CableTwoHalvePieceBalance extends AutoModeBase {
    double startTime;
    double previousTime;

    String path1 = "CABLE_COLLECT_CUBE_1";
    String path2 = "CABLE_SCORE_CUBE_1";
    String path3 = "CABLE_COLLECT_CUBE_2";
    String path4 = "CABLE_TWOHALVE_BALANCE";

    @Override
    protected void routine() throws AutoModeEndedException {
        startTime = Timer.getFPGATimestamp();
        previousTime = startTime;

        if (DriverStation.getAlliance() == DriverStation.Alliance.Red){
            path1 = "CABLE_COLLECT_CUBE_1_FLIPPED";
            path2 = "CABLE_SCORE_CUBE_1_FLIPPED";
            path3 = "CABLE_COLLECT_CUBE_2_FLIPPED";
            path4 = "CABLE_TWOHALVE_BALANCE_FLIPPED";
        }

        runAction(
                new ParallelAction(
                        new ScoreConeHighDriveAction(path1, false, Constants.MAX_VELOCITY, Constants.MAX_ACCELERATION),
                        new SeriesAction(
                                new WaitAction(1.5),
                                new SelectGamePieceAction(GamePieceHandler.GamePiece.CUBE),
                                new WaitForMarkerAction("open-intake",2),
                                new IntakeStartAction()
                        )
                )
        );

        runAction(
                new ParallelAction(
                        new DrivePathAction(path2),
                        new SeriesAction(
                                new SetIntakeAction(IntakeManager.ControlState.ROLLING),
                                new DexerPickupAction(2.3),
                                new WaitForMarkerAction("extend-arm", 1),
                                new SetArmAction(ArmManager.PositionState.HIGH)
                        )
                )
        );

        runAction(new GripperShootAction());
        printTime("scored 2nd piece in ");


        runAction(
                new ParallelAction(
                        new DrivePathAction(path3),
                        new SeriesAction(
                                new WaitAction(0.1),
                                new SetArmAction(ArmManager.PositionState.SPINDEXER_RETRACTED)
                        ),
                        new SeriesAction(
                                new WaitForMarkerAction("open-intake",2),
                                new IntakeStartAction()
                        )
                )
        );

        runAction(
                new ParallelAction(
                        new SeriesAction(
                                new DrivePathAction(path4, false, Constants.CHARGE_VELOCITY, Constants.MAX_ACCELERATION),
                                new IMUBalanceAction(Constants.AUTO_BALANCE_VELOCITY, Constants.AUTO_BALANCE_MAX_STOP_ERROR, Constants.AUTO_BALANCE_KP, startTime),
                                new WaitAction(0.1)
                        ),
                        new SeriesAction(
                                new SetIntakeAction(IntakeManager.ControlState.ROLLING),
                                new WaitAction(2),
                                new SetIntakeAction(IntakeManager.ControlState.DISABLED)
                        )
                )
        );

        printTime("autonomous done in ");
    }

    private void printTime(String message) {
        double timestamp = Timer.getFPGATimestamp();
        double timeDelta = timestamp - previousTime;
        double totalDelta = timestamp - startTime;

        previousTime = timestamp;
        System.out.printf("%s %5.3fs (%5.3fs)%n", message, timeDelta, totalDelta);
    }
}
