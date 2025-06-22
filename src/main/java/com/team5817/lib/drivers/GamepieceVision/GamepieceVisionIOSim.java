package com.team5817.lib.drivers.GamepieceVision;

import org.ironmaple.simulation.SimulatedArena;

import com.team254.lib.geometry.Pose2d;
import com.team5817.frc2025.subsystems.GamePieceVision.GamepieceVision.RobotPoseSupplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;

/** Simulated IO implementation using MapleSim gamepiece simulation. */
public class GamepieceVisionIOSim implements GamepieceVisionIO {
  private final RobotPoseSupplier mRobotPoseSupplier;
  private final SimulatedArena mSimulatedArena;

  public GamepieceVisionIOSim(RobotPoseSupplier robotPoseSupplier, SimulatedArena mSimulatedArena) {
    this.mSimulatedArena = mSimulatedArena;
    this.mRobotPoseSupplier = robotPoseSupplier;
  }

  @Override
  public void updateInputs(GamepieceVisionIOInputs inputs) {
    Pose2d robotPose = mRobotPoseSupplier.getPose();
    Translation2d robotTranslation = robotPose.getTranslation().wpi();
    edu.wpi.first.math.geometry.Rotation2d robotRotation = robotPose.getRotation().wpi();

    for (Pose3d gamepiece : mSimulatedArena.getGamePiecesByType("Coral")) {
      Translation2d gamepieceTranslation = gamepiece.toPose2d().getTranslation();

      Translation2d delta = gamepieceTranslation.minus(robotTranslation);
      Translation2d relativeToRobot = delta.rotateBy(robotRotation.unaryMinus());

      if (relativeToRobot.getX() < 0 
          && relativeToRobot.getNorm() <= 1.5 
          && Math.abs(relativeToRobot.getX())>Math.abs(relativeToRobot.getY())*2) {
        Translation3d robotToTarget = new Translation3d(
          relativeToRobot.getX(),
          relativeToRobot.getY(),
          0.0
        );

        inputs.data = new GamepieceVisionIOData(robotToTarget, Timer.getTimestamp());
        return; // Only report the first valid detection
      }
    }

    // No visible gamepieces
    inputs.data = new GamepieceVisionIOData(null, Timer.getTimestamp());
  }
}
