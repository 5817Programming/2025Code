package com.team5817.lib.swerve;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.path.PathPlannerPath;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.swerve.ChassisSpeeds;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team5817.frc2025.subsystems.Drive.SwerveConstants;
import com.team5817.lib.motion.PPPathPointState;

import edu.wpi.first.wpilibj.Timer;

public class DriveMotionPlanner {
  // Pure Pursuit Constants
  public static final double kPathLookaheadTime = 0.1; // From 1323 (2019)
  public static final double kPathMinLookaheadDistance = 0.3; // From 1323 (2019)
  public static final double kAdaptivePathMinLookaheadDistance = 0.15;
  public static final double kAdaptivePathMaxLookaheadDistance = 0.61;
  public static final double kAdaptiveErrorLookaheadCoefficient = 0.01;

  public enum FollowerType {
    PID
  }

  FollowerType mFollowerType = FollowerType.PID;

  public void setFollowerType(FollowerType type) {
    mFollowerType = type;
  }

  TrajectoryIterator mCurrentTrajectory;
  boolean mIsReversed = false;
  double mLastTime = Double.POSITIVE_INFINITY;
  PPPathPointState mLastSetpoint = null;
  PPPathPointState mSetpoint = new PPPathPointState();
  Pose2d mError = Pose2d.identity();

  Translation2d mTranslationalError = Translation2d.identity();
  Rotation2d mPrevHeadingError = Rotation2d.identity();
  Pose2d mCurrentState = Pose2d.identity();

  double mCurrentTrajectoryLength = 0.0;
  double mTotalTime = Double.POSITIVE_INFINITY;
  double mStartTime = Double.POSITIVE_INFINITY;
  ChassisSpeeds mOutput = new ChassisSpeeds();
  boolean mPathIsFinished = false;
  double startTime = 0;
  double timeout = Double.POSITIVE_INFINITY;

  // PID controllers for path following
  double mDt = 0.0;

  public DriveMotionPlanner() {
  }

  public void setTrajectory(final TrajectoryIterator trajectory, double timeout) {
    mCurrentTrajectory = trajectory;
    mSetpoint = trajectory.getCurrentState();
    mLastSetpoint = null;
    mCurrentTrajectoryLength = mCurrentTrajectory.getTimeView().last_interpolant();
    this.timeout = timeout;
  }

  public void reset() {
    mTranslationalError = Translation2d.identity();
    mPrevHeadingError = Rotation2d.identity();
    mLastSetpoint = null;
    mLastTime = Double.POSITIVE_INFINITY;
    mPathIsFinished = false;
    startTime = Timer.getTimestamp();
  }

  public ChassisSpeeds updatePIDChassis(ChassisSpeeds chassisSpeeds) {
    final double kPathk = 4;
    Twist2d pid_error = Pose2d.log(mError);
    chassisSpeeds.vxMetersPerSecond = (chassisSpeeds.vxMetersPerSecond) + kPathk * pid_error.dx;
    chassisSpeeds.vyMetersPerSecond = (chassisSpeeds.vyMetersPerSecond) + kPathk * pid_error.dy;

    chassisSpeeds.omegaRadiansPerSecond = chassisSpeeds.omegaRadiansPerSecond + kPathk * pid_error.dtheta;

    return chassisSpeeds;
  }

  public ChassisSpeeds update(double timestamp, Pose2d current_state) {

    if (mCurrentTrajectory == null)
      return null;

    if (!Double.isFinite(mLastTime))
      mLastTime = timestamp;

    mDt = timestamp - mLastTime;
    mLastTime = timestamp;
    PPPathPointState sample_point;
    mCurrentState = current_state;

    // Compute error in robot frame
    mPrevHeadingError = mError.getRotation();

    var translationalError = mCurrentState.getTranslation().inverse().translateBy(mSetpoint.getPose().getTranslation());
    var rotationalError = mSetpoint.getPose().getRotation().rotateBy(mCurrentState.getRotation().inverse());
    mError = new Pose2d(translationalError, rotationalError);

    sample_point = mCurrentTrajectory.advance(mDt);
    mSetpoint = sample_point;
    Logger.recordOutput("Following Pose", (mSetpoint.getPose().wpi()));
    var chassis_speeds = new ChassisSpeeds(
        sample_point.getXVel(),
        sample_point.getYVel(),
        sample_point.getThetaVel());
    mOutput = updatePIDChassis(chassis_speeds);

    mPathIsFinished = distance(current_state, mCurrentTrajectoryLength) < SwerveConstants.kTrajectoryDeadband ||
        Timer.getTimestamp() - startTime > timeout;
    mOutput = ChassisSpeeds.fromFieldRelativeSpeeds(mOutput, current_state.getRotation());

    return mOutput;
  }

  private double distance(Pose2d current_state, double additional_progress) {
    return mCurrentTrajectory
        .preview(additional_progress)
        .getPose()
        .distance(current_state);
  }

  public PathPlannerPath getPath() {
    if (mCurrentTrajectory == null)
      return null;
    return mCurrentTrajectory.getPath();
  }

  public Pose2d getEndPosition() {
    return mCurrentTrajectory.getTimeView().sample(mCurrentTrajectoryLength).getPose();
  }

  public Translation2d getTranslationalError() {
    return new Translation2d(
        mError.getTranslation().x(), mError.getTranslation().y());
  }

  public Rotation2d getHeadingError() {
    return mError.getRotation();
  }

  public boolean isPathFinished() {
    return mPathIsFinished;
  }

  public PPPathPointState getSetpoint() {
    return mSetpoint;
  }

}
