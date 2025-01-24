package com.team5817.lib.swerve;



import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.swerve.ChassisSpeeds;
import com.team254.lib.trajectory.TrajectoryIterator;
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
		PID,
		PURE_PURSUIT,
	}

	FollowerType mFollowerType = FollowerType.PID;
	public void setFollowerType(FollowerType type) {
		mFollowerType = type;
	}


	TrajectoryIterator mCurrentTrajectory;
	boolean mIsReversed = false;
	double mLastTime = Double.POSITIVE_INFINITY;
	public PPPathPointState mLastSetpoint = null;
	public PPPathPointState mSetpoint = new PPPathPointState();
	Pose2d mError = Pose2d.identity();

	SwerveHeadingController mHeadingController = new SwerveHeadingController();
	Translation2d mTranslationalError = Translation2d.identity();
	Rotation2d mPrevHeadingError = Rotation2d.identity();
	Pose2d mCurrentState = Pose2d.identity();

	double mCurrentTrajectoryLength = 0.0;
	double mTotalTime = Double.POSITIVE_INFINITY;
	double mStartTime = Double.POSITIVE_INFINITY;
	ChassisSpeeds mOutput = new ChassisSpeeds();

	// PID controllers for path following
	double mDt = 0.0;

	public DriveMotionPlanner() {}

	public void setTrajectory(final TrajectoryIterator trajectory) {
		mCurrentTrajectory = trajectory;
		mSetpoint = trajectory.getCurrentState();
		mLastSetpoint = null;
		mCurrentTrajectoryLength =
				mCurrentTrajectory.getTimeView().last_interpolant();
	}

	public void reset() {
		mTranslationalError = Translation2d.identity();
		mPrevHeadingError = Rotation2d.identity();
		mLastSetpoint = null;
		mOutput = new ChassisSpeeds();
		mLastTime = Double.POSITIVE_INFINITY;
	}

	protected ChassisSpeeds updatePIDChassis(ChassisSpeeds chassisSpeeds) {
		// Feedback on longitudinal error (distance).
		final double kPathk = 2.4;/* * Math.ypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)*/;//0.15;
		Twist2d pid_error = Pose2d.log(mError);
		chassisSpeeds.vxMetersPerSecond = (chassisSpeeds.vxMetersPerSecond) - kPathk * pid_error.dx;
		chassisSpeeds.vyMetersPerSecond = (chassisSpeeds.vyMetersPerSecond) + kPathk * pid_error.dy;
		// chassisSpeeds.vxMetersPerSecond = (chassisSpeeds.vxMetersPerSecond * 1) ;
		// chassisSpeeds.vyMetersPerSecond = (chassisSpeeds.vyMetersPerSecond * 1) ;
	
		// chassisSpeeds.omegaRadiansPerSecond = mHeadingController.update(mError.getRotation().inverse(), Timer.getTimestamp());TODO
		// chassisSpeeds.omegaRadiansPerSecond = 0;
		return chassisSpeeds;
	}

	public ChassisSpeeds update(double timestamp, Pose2d current_state) {
		if (mCurrentTrajectory == null) return null;

		if (!Double.isFinite(mLastTime)) mLastTime = timestamp;
		mDt = timestamp - mLastTime;
		mLastTime = timestamp;
		PPPathPointState sample_point;
		mCurrentState = current_state;

			// Compute error in robot frame
			mPrevHeadingError = mError.getRotation();
			mError = current_state.inverse().transformBy(mSetpoint.getPose());
		


 				sample_point = mCurrentTrajectory.advance(mDt);
				mSetpoint = sample_point;

				final double velocity_m = mSetpoint.getVelocity();
				// Field relative
				var course = mSetpoint.getCourse();
				Rotation2d motion_direction = course;
				// Adjust course by ACTUAL heading rather than planned to decouple heading and translation errors.

				var chassis_speeds = new ChassisSpeeds(
						velocity_m * motion_direction.cos(),
						velocity_m * motion_direction.sin(),
						0
						);
				mOutput = updatePIDChassis(chassis_speeds);
	                   
		return mOutput;
	}

	public Pose2d getEndPosition() {
		return mCurrentTrajectory.getTimeView().sample(mCurrentTrajectoryLength).getPose();
	}

	public synchronized Translation2d getTranslationalError() {
		return new Translation2d(
				mError.getTranslation().x(), mError.getTranslation().y());
	}

	public synchronized Rotation2d getHeadingError() {
		return mError.getRotation();
	}

	private double distance(Pose2d current_state, double additional_progress) {
		return mCurrentTrajectory
				.preview(additional_progress)
				.getPose()
				.distance(current_state);
	}

	public synchronized PPPathPointState getSetpoint() {
		return mSetpoint;
	}


}