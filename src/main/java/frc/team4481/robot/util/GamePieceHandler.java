package frc.team4481.robot.util;

public class GamePieceHandler {
    public static GamePieceHandler instance = null;
    private GamePiece gamePiece = GamePiece.CONE;

    private GamePieceHandler(){}

    public static GamePieceHandler getInstance(){
        if(instance == null){
            instance = new GamePieceHandler();
        }

        return instance;
    }

    public enum GamePiece {
        CONE,
        CUBE
    }

    public GamePiece getGamePiece() {
        return gamePiece;
    }

    public void setGamePiece(GamePiece gamePiece) {
        this.gamePiece = gamePiece;
    }
}
