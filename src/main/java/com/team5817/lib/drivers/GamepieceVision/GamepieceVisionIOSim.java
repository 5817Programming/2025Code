package com.team5817.lib.drivers.GamepieceVision;

import java.util.List;

import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;

import com.team254.lib.geometry.Pose2d;
import com.team5817.frc2025.subsystems.GamePieceVision.GamepieceVision.RobotPoseSupplier;
import com.team5817.lib.util.AllianceFlipUtil;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
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

    List<Pose3d> gamepieces = mSimulatedArena.getGamePiecesByType("Coral");

    Pose3d base = new Pose3d(2, .583, 0, Rotation3d.kZero);

    Pose3d cornerBL = base;
    Pose3d cornerBR = AllianceFlipUtil.apply(base, true, false);
    Pose3d cornerTL = AllianceFlipUtil.apply(base, false, true);
    Pose3d cornerTR = AllianceFlipUtil.apply(base, true, true);

    gamepieces.add(cornerBL);
    gamepieces.add(cornerBR);
    gamepieces.add(cornerTL);
    gamepieces.add(cornerTR);

    Logger.recordOutput("GamepieceVision/Sim Coral", gamepieces.toArray(new Pose3d[0]));

    for (Pose3d gamepiece : gamepieces) {
      Translation2d gamepieceTranslation = gamepiece.toPose2d().getTranslation();

      Translation2d delta = gamepieceTranslation.minus(robotTranslation);
      Translation2d relativeToRobot = delta.rotateBy(robotRotation.unaryMinus());

      if (relativeToRobot.getX() < 0 
          && relativeToRobot.getNorm() <= 2
          && Math.abs(relativeToRobot.getX())>Math.abs(relativeToRobot.getY())) {
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
