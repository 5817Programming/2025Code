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

  private PIDController mXController = new PIDController(4, 0, 0);
  private PIDController mYController = new PIDController(4, 0, 0);
  private SwerveHeadingController mThetaController;

  boolean mAutoAlignComplete = false;

  private Pose2d mFieldToTargetPoint;
  private Pose2d poseDeadband;
  private OptionalDouble mStartTime;
  private Translation2d error = new Translation2d();

  /**
   * Constructor for AutoAlignMotionPlanner.
   */
  public AutoAlignMotionPlanner(SwerveHeadingController mThetaController) {
    this.mThetaController = mThetaController;

  }

  /**
   * Resets the motion planner to its initial state.
   */
  public void reset() {
    mStartTime = OptionalDouble.of(Timer.getFPGATimestamp());
    mXController.reset();
    mYController.reset();
    mAutoAlignComplete = false;
  }

  /**
   * Sets the target point for auto-alignment.
   * 
   * @param targetPoint The target point to align to.
   */
  public void setTargetPoint(Pose2d targetPoint, Pose2d poseDeadband) {
    mFieldToTargetPoint = targetPoint;
    mXController.reset();
    mYController.reset();
    this.poseDeadband = Pose2d.fromTranslation(poseDeadband.getTranslation()).withRotation(poseDeadband.getRotation());
    Logger.recordOutput("Align Point", new edu.wpi.first.math.geometry.Pose2d(
        mFieldToTargetPoint.getTranslation().wpi(), mFieldToTargetPoint.getRotation().wpi()));
  }

  /**
   * Updates the motion planner with the current timestamp, pose, and velocity.
   * 
   * @param timestamp    The current timestamp.
   * @param current_pose The current pose of the robot.
   * @param current_vel  The current velocity of the robot.
   * @return The updated chassis speeds.
   */
  public ChassisSpeeds updateAutoAlign(double timestamp, Pose2d current_pose) {
    mXController.setSetpoint(mFieldToTargetPoint.getTranslation().x());
    mYController.setSetpoint(mFieldToTargetPoint.getTranslation().y());

    mThetaController.setSnapTarget(mFieldToTargetPoint.getRotation());
    double currentRotation = current_pose.getRotation().getRadians();

    if (mFieldToTargetPoint.getRotation().getRadians() - currentRotation > Math.PI) {
      currentRotation += 2 * Math.PI;
    } else if (mFieldToTargetPoint.getRotation().getRadians() - currentRotation < -Math.PI) {
      currentRotation -= 2 * Math.PI;
    }
    double xOutput = mXController.calculate(current_pose.getTranslation().x());
    double yOutput = mYController.calculate(current_pose.getTranslation().y());
    double thetaOutput = mThetaController.update(current_pose.getRotation(), timestamp);
    ChassisSpeeds setpoint = new ChassisSpeeds();
    xOutput = Math.copySign(Math.min(Math.abs(xOutput), 1.3), xOutput);
    yOutput = Math.copySign(Math.min(Math.abs(yOutput), 1.3), yOutput);
    this.error = current_pose.minus(mFieldToTargetPoint).getTranslation();
    boolean thetaWithinDeadband = current_pose.getRotation().distance(mFieldToTargetPoint.getRotation()) < poseDeadband
        .getRotation().getRadians() && Math.abs(thetaOutput) < 0.02;
    boolean xWithinDeadband = Math.abs(mXController.getSetpoint() - current_pose.getTranslation().x()) < poseDeadband
        .getTranslation().x();
    boolean yWithinDeadband = Math.abs(mYController.getSetpoint() - current_pose.getTranslation().y()) < poseDeadband
        .getTranslation().y();
    if (mAutoAlignComplete)
      setpoint = ChassisSpeeds.fromFieldRelativeSpeeds(
          0.0,
          0.0,
          0.0,
          current_pose.getRotation());
    setpoint = ChassisSpeeds.fromFieldRelativeSpeeds(
        xWithinDeadband ? 0.0 : xOutput,
        yWithinDeadband ? 0.0 : yOutput,
        thetaWithinDeadband ? 0.0 : thetaOutput,
        current_pose.getRotation());
    mAutoAlignComplete = thetaWithinDeadband && xWithinDeadband && yWithinDeadband;
    Logger.recordOutput("AutoAlign/xDone", xWithinDeadband);
    Logger.recordOutput("AutoAlign/yDone", yWithinDeadband);
    Logger.recordOutput("AutoAlign/tDone", thetaWithinDeadband);

    if (mStartTime.isPresent() && mAutoAlignComplete) {
      System.out.println("Auto align took: " + (Timer.getFPGATimestamp() - mStartTime.getAsDouble()));
      mStartTime = OptionalDouble.empty();
    }
    return setpoint;
  }

  /**
   * Checks if the auto-alignment is complete.
   * 
   * @return True if auto-alignment is complete, false otherwise.
   */
  public boolean getAutoAlignComplete() {
    return mAutoAlignComplete;
  }

  public Translation2d getAutoAlignError() {
    return this.error;
  }
}
