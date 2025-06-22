package com.team5817.lib.drivers.GamepieceVision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Translation3d;



public interface GamepieceVisionIO {
  default void updateInputs(GamepieceVisionIOInputs inputs){}

  @AutoLog
  public class GamepieceVisionIOInputs {
    public GamepieceVisionIOData data = new GamepieceVisionIOData(new Translation3d(), 0.0);
  }

  record GamepieceVisionIOData(
      Translation3d robotToTarget,
      double timestamp) {
  }

}