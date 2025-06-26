package com.team5817.frc2025.subsystems.Drive;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.swerve.ChassisSpeeds;
import com.team5817.lib.swerve.SwerveHeadingController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;

import java.util.OptionalDouble;

import org.littletonrobotics.junction.Logger;

/**
 * Class responsible for planning the motion for auto-alignment.
 */
public class AutoAlignMotionPlanner {

  private final PIDController distanceController = new PIDController(5.6, 0.0, 0.0);
  private final SwerveHeadingController mThetaController;

  private boolean mAutoAlignComplete = false;

  private Pose2d mFieldToTargetPoint;
  private Pose2d poseDeadband;
  private OptionalDouble mStartTime = OptionalDouble.empty();
  private Translation2d error = new Translation2d();

  public AutoAlignMotionPlanner(SwerveHeadingController mThetaController) {
    this.mThetaController = mThetaController;
  }

  public void reset() {
    mStartTime = OptionalDouble.of(Timer.getFPGATimestamp());
    distanceController.reset();
    mAutoAlignComplete = false;
  }

  public void setTargetPoint(Pose2d targetPoint, Pose2d poseDeadband) {
    mFieldToTargetPoint = targetPoint;
    distanceController.reset();
    this.poseDeadband = Pose2d.fromTranslation(poseDeadband.getTranslation()).withRotation(poseDeadband.getRotation());

    Logger.recordOutput("AutoAlign/Point", new edu.wpi.first.math.geometry.Pose2d(
        mFieldToTargetPoint.getTranslation().wpi(), mFieldToTargetPoint.getRotation().wpi()));
  }

  public ChassisSpeeds updateAutoAlign(double timestamp, Pose2d current_pose) {
    Logger.recordOutput("AutoAlign/Valid Point", mFieldToTargetPoint != null);
    if (mFieldToTargetPoint == null) {
      return new ChassisSpeeds();
    }

    Translation2d translationError = mFieldToTargetPoint.getTranslation().minus(current_pose.getTranslation());
    this.error = translationError;
    double distanceError = translationError.norm();
    distanceController.setSetpoint(0.0);
    double driveSpeed = distanceController.calculate(distanceError);

    driveSpeed = Math.copySign(Math.min(Math.abs(driveSpeed), 2), driveSpeed);

    Translation2d driveVector = distanceError > 1e-4
        ? translationError.direction().flip().toTranslation().times(driveSpeed)
        : new Translation2d();

    mThetaController.setSnapTarget(mFieldToTargetPoint.getRotation());
    double thetaOutput = mThetaController.update(current_pose.getRotation(), timestamp);

    boolean translationWithinDeadband = distanceError < poseDeadband.getTranslation().norm();
    boolean rotationWithinDeadband =
        current_pose.getRotation().distance(mFieldToTargetPoint.getRotation()) < poseDeadband.getRotation().getRadians()
        && Math.abs(thetaOutput) < 0.02;

    if (mAutoAlignComplete) {
      return ChassisSpeeds.fromFieldRelativeSpeeds(0.0, 0.0, 0.0, current_pose.getRotation());
    } 

    mAutoAlignComplete = translationWithinDeadband && rotationWithinDeadband;

    Logger.recordOutput("AutoAlign/TranslationDone", translationWithinDeadband);
    Logger.recordOutput("AutoAlign/RotationDone", rotationWithinDeadband);

    if (mStartTime.isPresent() && mAutoAlignComplete) {
      System.out.println("Auto align took: " + (Timer.getFPGATimestamp() - mStartTime.getAsDouble()));
      mStartTime = OptionalDouble.empty();
    }

    return ChassisSpeeds.fromFieldRelativeSpeeds(
        translationWithinDeadband ? 0.0 : driveVector.x(),
        translationWithinDeadband ? 0.0 : driveVector.y(),
        rotationWithinDeadband ? 0.0 : thetaOutput,
        current_pose.getRotation());
  }

  public boolean getAutoAlignComplete() {
    return mAutoAlignComplete;
  }

  public Translation2d getAutoAlignError() {
    return error;
  }
}