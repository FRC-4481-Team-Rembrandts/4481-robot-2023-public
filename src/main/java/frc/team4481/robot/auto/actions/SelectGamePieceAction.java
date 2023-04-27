package frc.team4481.robot.auto.actions;

import frc.team4481.lib.auto.actions.Action;
import frc.team4481.robot.util.GamePieceHandler;

public class SelectGamePieceAction implements Action {
    private final GamePieceHandler gamePieceHandler = GamePieceHandler.getInstance();

    private GamePieceHandler.GamePiece gamePiece;

    public SelectGamePieceAction(GamePieceHandler.GamePiece gamePiece) {
        this.gamePiece = gamePiece;
    }

    @Override
    public void start() {
        gamePieceHandler.setGamePiece(gamePiece);
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void done() {

    }
}
