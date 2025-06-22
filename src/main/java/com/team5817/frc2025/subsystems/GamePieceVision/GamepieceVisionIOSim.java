package com.team5817.frc2025.subsystems.GamePieceVision;

import com.team254.lib.geometry.Pose2d;
import com.team5817.frc2025.subsystems.GamePieceVision.GamepieceVision.RobotPoseSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;

/** Simulated IO implementation for injecting fake gamepiece observations. */
public class GamepieceVisionIOSim implements GamepieceVisionIO {
  private final RobotPoseSupplier mRobotPoseSupplier;
  private Translation2d simulatedGamepiece = new Translation2d(3.0, 3.0);

  public GamepieceVisionIOSim(RobotPoseSupplier robotPoseSupplier) {
    this.mRobotPoseSupplier = robotPoseSupplier;
  }

  /** Injects a new simulated gamepiece pose on the field. */
  public void setSimulatedGamepiece(Translation2d pose) {
    simulatedGamepiece = pose;
  }

  @Override
  public void updateInputs(GamepieceVisionIOInputs inputs) {
    Pose2d robotPose = mRobotPoseSupplier.getPose();

    Translation2d robotTranslation = robotPose.getTranslation().wpi();
    edu.wpi.first.math.geometry.Rotation2d robotRotation = robotPose.getRotation().wpi();

    Translation2d relativeToRobot = simulatedGamepiece.minus(robotTranslation).rotateBy(robotRotation.unaryMinus());

    Translation3d robotToTarget = new Translation3d(relativeToRobot.getX(), relativeToRobot.getY(), 0.0);

    inputs.data = new GamepieceVisionIOData(robotToTarget, Timer.getTimestamp());
  }
}
