package com.team5817.frc2025.subsystems.Drive;

import com.team5817.frc2025.Constants;
import com.team5817.frc2025.Robot;
import com.team5817.frc2025.RobotState;
import com.team5817.frc2025.Constants.SwerveConstants;
import com.team5817.frc2025.Constants.SwerveConstants.Mod0;
import com.team5817.frc2025.Constants.SwerveConstants.Mod1;
import com.team5817.frc2025.Constants.SwerveConstants.Mod2;
import com.team5817.frc2025.Constants.SwerveConstants.Mod3;
import com.team5817.frc2025.loops.ILooper;
import com.team5817.frc2025.loops.Loop;
import com.team5817.frc2025.subsystems.Cancoders;
import com.team5817.frc2025.subsystems.WheelTracker;
import com.team5817.lib.Util;
import com.team5817.lib.drivers.Pigeon;
import com.team5817.lib.drivers.Subsystem;
import com.team5817.lib.swerve.DriveMotionPlanner;
import com.team5817.lib.swerve.DriveMotionPlanner.FollowerType;
import com.team5817.lib.swerve.SwerveHeadingController;
import com.team5817.lib.swerve.SwerveModule;
import com.team5817.lib.swerve.SwerveModulePosition;
import com.ctre.phoenix6.controls.StaticBrake;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithMotion;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.swerve.ChassisSpeeds;
import com.team254.lib.swerve.SwerveKinematicLimits;
import com.team254.lib.swerve.SwerveModuleState;
import com.team254.lib.swerve.SwerveSetpoint;
import com.team254.lib.swerve.SwerveSetpointGenerator;
import com.team254.lib.trajectory.timing.TimedState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Pound;
import static edu.wpi.first.units.Units.Volt;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends Subsystem {

	public enum DriveControlState {
		FORCE_ORIENT,
		OPEN_LOOP,
		HEADING_CONTROL,
		VELOCITY,
		PATH_FOLLOWING,
		AUTOALIGN
	}

	private WheelTracker mWheelTracker;
	private Pigeon mPigeon = Pigeon.getInstance();
	public SwerveModule[] mModules;

	private SwerveInputs mPeriodicIO = new SwerveInputs();
	private DriveControlState mControlState = DriveControlState.HEADING_CONTROL;

	private SwerveSetpointGenerator mSetpointGenerator;

	private boolean odometryReset = false;

	private final DriveMotionPlanner mMotionPlanner;
	private final AutoAlignMotionPlanner mAutoAlignMotionPlanner = new AutoAlignMotionPlanner();
	private final SwerveHeadingController mHeadingController;

	private Translation2d enableFieldToOdom = null;

	private boolean mOverrideTrajectory = false;
	private boolean mOverrideHeading = false;

	private Rotation2d mTrackingAngle = Rotation2d.identity();

	private SwerveKinematicLimits mKinematicLimits = SwerveConstants.kSwerveKinematicLimits;
	private SwerveKinematicLimits mUncappedKinematicLimits = SwerveConstants.kSwerveUncappedKinematicLimits;


	private static AutoAlignPointSelector.RequestedAlignment mAlignment = AutoAlignPointSelector.RequestedAlignment.AUTO;
	private static Drive mInstance;

	public static Drive getInstance() {
		if (mInstance == null) {
			mInstance = new Drive();
		}
		return mInstance;
	}
	    public static final DriveTrainSimulationConfig mapleSimConfig = DriveTrainSimulationConfig.Default()
            .withRobotMass(Pound.of(115))
            .withTrackLengthTrackWidth(Inches.of(29), Inches.of(29))
            .withGyro(COTS.ofPigeon2())
            .withSwerveModule(new SwerveModuleSimulationConfig(
                    DCMotor.getKrakenX60(1),
                    DCMotor.getFalcon500(1),
                    SwerveConstants.driveGearRatio,
                    SwerveConstants.angleGearRatio,
                    Volt.of(.2),
                    Volt.of(.2),
                    Inches.of(2),
                    KilogramSquareMeters.of(.005),
                    1.2));
		
		public static SwerveDriveSimulation driveSimulation;
		public static void registerDriveSimulation(SwerveDriveSimulation sim) {
			driveSimulation = sim;
		}
	private Drive() {
		mModules = new SwerveModule[] {
			new SwerveModule(
					0, Mod0.SwerveModuleConstants(), Cancoders.getInstance().getFrontLeft()),
			new SwerveModule(
					1, Mod1.SwerveModuleConstants(), Cancoders.getInstance().getFrontRight()),
			new SwerveModule(
					2, Mod2.SwerveModuleConstants(), Cancoders.getInstance().getBackLeft()),
			new SwerveModule(
					3, Mod3.SwerveModuleConstants(), Cancoders.getInstance().getBackRight())
		};


		mMotionPlanner = new DriveMotionPlanner();
		mHeadingController = new SwerveHeadingController();

		mMotionPlanner.reset();
		mPigeon.setYaw(0.0);
		mWheelTracker = new WheelTracker(mModules);
		mSetpointGenerator = new SwerveSetpointGenerator(SwerveConstants.kKinematics);
		mHeadingController.setStabilizeTarget(mPigeon.getYaw());
	}

	public void setKinematicLimits(SwerveKinematicLimits newLimits) {
		this.mKinematicLimits = newLimits;
	}

	/**
	 * Updates drivetrain with latest desired speeds from the joystick, and sets DriveControlState appropriately.
	 *
	 * @param speeds ChassisSpeeds object derived from joystick input
	 */
	public void feedTeleopSetpoint(ChassisSpeeds speeds) {
		double omega = mHeadingController.update(mPeriodicIO.heading, Timer.getTimestamp());
		if(mControlState != DriveControlState.HEADING_CONTROL&&Math.abs(mPeriodicIO.des_chassis_speeds.omegaRadiansPerSecond-omega) > .1){
			mHeadingController.setStabilizeTarget(mPigeon.getYaw());
		}

		if (mControlState == DriveControlState.PATH_FOLLOWING) {
			if (Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
					> mKinematicLimits.kMaxDriveVelocity * 0.1) {
				mControlState = DriveControlState.OPEN_LOOP;
			} else {
				return;
			}
		}
		if (mControlState == DriveControlState.AUTOALIGN) {
			if (Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
					> mKinematicLimits.kMaxDriveVelocity * 0.1) {
				mControlState = DriveControlState.OPEN_LOOP;
			} else {	
				ChassisSpeeds speed = mAutoAlignMotionPlanner.updateAutoAlign(mPeriodicIO.timestamp, RobotState.getInstance().getGlobalPose(mPeriodicIO.timestamp).withRotation(mPeriodicIO.heading),
																	mPeriodicIO.predicted_velocity);
				if(speed != null){
					mPeriodicIO.des_chassis_speeds = speed;
				}
				return;
			}
		}
		
		if (mControlState == DriveControlState.OPEN_LOOP||mControlState == DriveControlState.HEADING_CONTROL) {
		
				double x = speeds.vxMetersPerSecond;
				double y = speeds.vyMetersPerSecond;

				if (Math.abs(speeds.omegaRadiansPerSecond) > .1) {
					omega = speeds.omegaRadiansPerSecond;
					
				}
					
				mPeriodicIO.des_chassis_speeds = new ChassisSpeeds(x, y, omega);
				return;
			
		} else if (mControlState != DriveControlState.OPEN_LOOP) {
			mControlState = DriveControlState.OPEN_LOOP;
		
		}


		mPeriodicIO.des_chassis_speeds = speeds;
		
	}

	public void setOpenLoop(ChassisSpeeds speeds) {
		mPeriodicIO.des_chassis_speeds = speeds;
		if (mControlState != DriveControlState.OPEN_LOOP) {
			mControlState = DriveControlState.OPEN_LOOP;
		}
	}
	public void setVelocity(ChassisSpeeds speeds) {
		mPeriodicIO.des_chassis_speeds = speeds;
		if (mControlState != DriveControlState.VELOCITY) {
			mControlState = DriveControlState.VELOCITY;
		}
	}

	/**
	 * Instructs the drivetrain to snap heading to target angle.
	 *
	 * @param angle Rotation2d angle to snap to.
	 */
	public void snapHeading(Rotation2d angle) {
		if (mControlState != DriveControlState.HEADING_CONTROL && mControlState != DriveControlState.PATH_FOLLOWING) {
			mControlState = DriveControlState.HEADING_CONTROL;
		}
		mHeadingController.setSnapTarget(angle);
	}

	/**`
	 * Instructs the drivetrain to stabilize heading around target angle.
	 *
	 * @param angle Target angle to stabilize around.
	 */
	public void stabilizeHeading(Rotation2d angle) {
		if (mControlState != DriveControlState.HEADING_CONTROL && mControlState != DriveControlState.PATH_FOLLOWING) {
			mControlState = DriveControlState.HEADING_CONTROL;
		}
		mHeadingController.setStabilizeTarget(angle);
		
	}

	public synchronized void setAlignment(AutoAlignPointSelector.RequestedAlignment alignment){
		mAlignment = alignment;
	}

	/**
	 * Enable/disables vision heading control.
	 *
	 * @param value Whether or not to override rotation joystick with vision target. 
	 */
	public synchronized void overrideHeading(boolean value) {
		mOverrideHeading = value;
	}

	/**
	 * Updates needed angle to track a goal.
	 *
	 * @param angle Sets the wanted robot heading to track a goal.
	 */
	public synchronized void feedTrackingSetpoint(Rotation2d angle) {
		mTrackingAngle = angle;
	}

	/**
	 * Stops modules in place.
	 */
	public synchronized void stopModules() {
		List<Rotation2d> orientations = new ArrayList<>();
		for (SwerveModuleState SwerveModuleState : mPeriodicIO.des_module_states) {
			orientations.add(SwerveModuleState.angle);
		}
		orientModules(orientations);
	}

	/**
	 * Orients modules to the angles provided.
	 * @param orientations Rotation2d of target angles, indexed by module number.
	 */
	public synchronized void orientModules(List<Rotation2d> orientations) {
		if (mControlState != DriveControlState.FORCE_ORIENT) {
			mControlState = DriveControlState.FORCE_ORIENT;
		}
		for (int i = 0; i < mModules.length; ++i) {
			mPeriodicIO.des_module_states[i] = new SwerveModuleState(0.0, orientations.get(i));
		}
	}
	boolean autoAlignFinishedOverrride = false;
	public void autoAlign(){
		autoAlignFinishedOverrride = false;
		Optional<Pose2d> targetPoint = AutoAlignPointSelector.chooseTargetPoint(getPose(), mAlignment);
		if(targetPoint.isEmpty()){
			return;
		}
		Logger.recordOutput("Drive/TargetPoint", targetPoint.get().wpi());
		mAutoAlignMotionPlanner.setTargetPoint(targetPoint.get());
		if (mControlState != DriveControlState.AUTOALIGN) {
			mAutoAlignMotionPlanner.reset();
			mControlState = DriveControlState.AUTOALIGN;
		}
	}	

	public boolean getAutoAlignComplete(){
		if(autoAlignFinishedOverrride)
			return true;
		return mAutoAlignMotionPlanner.getAutoAlignComplete();
	}
	public void autoAlignFinishedOverrride(){
		autoAlignFinishedOverrride = true;
	}

	@Override
	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {
				mPeriodicIO.des_chassis_speeds = new ChassisSpeeds();
				mControlState = DriveControlState.OPEN_LOOP;
				mWheelTracker.start();
			}

			@Override
			public void onLoop(double timestamp) {
				synchronized (Drive.this) {
					switch (mControlState) {
						case PATH_FOLLOWING:
							// updatePathFollower();
							break;
						case AUTOALIGN:
							break;
						case HEADING_CONTROL:
							break;
						case OPEN_LOOP:
						case VELOCITY:
						case FORCE_ORIENT:
							break;
						default:
							stop();
							break;
					}

					updateSetpoint();
					
					RobotState.getInstance()
							.addOdometryUpdate(
									timestamp,
									mWheelTracker.getRobotPose(),
									mPeriodicIO.measured_velocity,
									mPeriodicIO.predicted_velocity);
					mWheelTracker.readPeriodicInputs();
				}

			}

			@Override
			public void onStop(double timestamp) {
				mPeriodicIO.des_chassis_speeds = new ChassisSpeeds();
				mControlState = DriveControlState.OPEN_LOOP;
				mWheelTracker.stop();
				enableFieldToOdom = null;
			}
		});
	}

	@Override
	public void readPeriodicInputs() {
		SwerveModuleState[] module_states = new SwerveModuleState[4];
		if(!Robot.isReal()&&Constants.mode == Constants.Mode.SIM){
			mPeriodicIO.heading = new Rotation2d(driveSimulation.getSimulatedDriveTrainPose().getRotation());
			mPeriodicIO.pitch = Rotation2d.identity();
			for (int i = 0; i < mModules.length; i++) {
				module_states[i] = new SwerveModuleState(driveSimulation.getModules()[i].getCurrentState());
			}
		}else{
			for (SwerveModule swerveModule : mModules) {
				swerveModule.readPeriodicInputs();
			}
			mPeriodicIO.heading = mPigeon.getYaw();
			mPeriodicIO.pitch = mPigeon.getPitch();
			module_states = getModuleStates();
		}		

		mPeriodicIO.timestamp = Timer.getFPGATimestamp();
		Twist2d twist_vel = Constants.SwerveConstants.kKinematics
				.toChassisSpeeds(module_states)
				.toTwist2d();
		Translation2d translation_vel = new Translation2d(twist_vel.dx, twist_vel.dy);
		translation_vel = translation_vel.rotateBy(getHeading());
		mPeriodicIO.measured_velocity = new Twist2d(
				translation_vel.getTranslation().x(),
				translation_vel.getTranslation().y(),
				twist_vel.dtheta);
		mPeriodicIO.timestamp = Timer.getFPGATimestamp();

	}

	/**
	 * @param pid_enable Switches between using PID control or Pure Pursuit control to follow trajectories.
	 */
	public synchronized void setUsePIDControl(boolean pid_enable) {
		if (pid_enable) {
			mMotionPlanner.setFollowerType(FollowerType.PID);
		} else {
			mMotionPlanner.setFollowerType(FollowerType.PURE_PURSUIT);
		}
	}


	/**
	 * Exits trajectory following early.
	 * @param value Whether to override the current trajectory.
	 */
	public synchronized void overrideTrajectory(boolean value) {
		mOverrideTrajectory = value;
	}


	/**
	 * Updates the wanted setpoint, including whether heading should
	 * be overridden to the tracking angle. Also includes
	 * updates for Path Following.
	 */
	private void updateSetpoint() {
		if (mControlState == DriveControlState.FORCE_ORIENT) return;

		Pose2d robot_pose_vel = new Pose2d(
				mPeriodicIO.des_chassis_speeds.vxMetersPerSecond * Constants.kLooperDt * 4.0,
				mPeriodicIO.des_chassis_speeds.vyMetersPerSecond * Constants.kLooperDt * 4.0,
				Rotation2d.fromRadians(
						mPeriodicIO.des_chassis_speeds.omegaRadiansPerSecond * Constants.kLooperDt * 4.0));

		Twist2d twist_vel = Pose2d.log(robot_pose_vel).scaled(1.0 / (4.0 * Constants.kLooperDt));

		ChassisSpeeds wanted_speeds;
		if (mOverrideHeading) {
			stabilizeHeading(mTrackingAngle);
			double new_omega = mHeadingController.update(mPigeon.getYaw(), Timer.getTimestamp());
			ChassisSpeeds speeds = new ChassisSpeeds(twist_vel.dx, twist_vel.dy, new_omega);
			wanted_speeds = speeds;
		} else {
			wanted_speeds = new ChassisSpeeds(twist_vel.dx, twist_vel.dy, twist_vel.dtheta);
		}

		mPeriodicIO.setpoint = mSetpointGenerator.generateSetpoint(mKinematicLimits, mPeriodicIO.setpoint, wanted_speeds, Constants.kLooperDt);
		var uncapped_setpoint = mSetpointGenerator.generateSetpoint(mUncappedKinematicLimits, mPeriodicIO.setpoint, wanted_speeds, Constants.kLooperDt);
		mPeriodicIO.predicted_velocity = wanted_speeds.toTwist2d();
				// Pose2d.log(Pose2d.exp(wanted_speeds.toTwist2d()).rotateBy(getHeading()));

		mPeriodicIO.uncapped_module_states = uncapped_setpoint.mModuleStates;
		mPeriodicIO.des_module_states = mPeriodicIO.setpoint.mModuleStates;
		
	}

	public void resetModulesToAbsolute() {
		for (SwerveModule module : mModules) {
			module.resetToAbsolute();
		}
	}

	public void zeroGyro() {
		zeroGyro(0.0);
	}

	public void zeroGyro(double reset_deg) {
		mPigeon.setYaw(reset_deg);
		enableFieldToOdom = null;
	}

	/**
	 * Configs if module drive motors should brake when commanded neutral output.
	 * @param brake Enable brake
	 */
	public void setNeutralBrake(boolean brake) {
		for (SwerveModule swerveModule : mModules) {
			swerveModule.setDriveNeutralBrake(brake);
		}
	}

	@Override
	public void writePeriodicOutputs() {
		
		for (int i = 0; i < mModules.length; i++) {
			if (mControlState == DriveControlState.OPEN_LOOP || mControlState == DriveControlState.HEADING_CONTROL) {
				mModules[i].setOpenLoop(mPeriodicIO.des_module_states[i]);
			}else if (mControlState == DriveControlState.PATH_FOLLOWING
					|| mControlState == DriveControlState.VELOCITY
					|| mControlState == DriveControlState.FORCE_ORIENT
					|| mControlState == DriveControlState.AUTOALIGN) {
				mModules[i].setVelocity(mPeriodicIO.des_module_states[i]);
			}
		}
		if(!Robot.isReal()&&Constants.mode == Constants.Mode.SIM){
			driveSimulation.setRobotSpeeds(mPeriodicIO.setpoint.mChassisSpeeds.wpi());
		}else{
			for (SwerveModule swerveModule : mModules) {
				swerveModule.writePeriodicOutputs();
		}}
	
	}

	public SwerveModuleState[] getModuleStates() {
		SwerveModuleState[] states = new SwerveModuleState[4];
		for (SwerveModule mod : mModules) {
			states[mod.moduleNumber()] = mod.getState();
		}
		return states;
	}

	public SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] states = new SwerveModulePosition[4];
		for (SwerveModule mod : mModules) {
			states[mod.moduleNumber()] = mod.getPosition();
		}
		return states;
	}

	public edu.wpi.first.math.kinematics.SwerveModulePosition[] getWpiModulePositions() {
		edu.wpi.first.math.kinematics.SwerveModulePosition[] states =
				new edu.wpi.first.math.kinematics.SwerveModulePosition[4];
		for (SwerveModule mod : mModules) {
			states[mod.moduleNumber()] = mod.getWpiPosition();
		}
		return states;
	}
	public edu.wpi.first.math.kinematics.SwerveModuleState[] getWpiModuleStates() {
		edu.wpi.first.math.kinematics.SwerveModuleState[] states =
				new edu.wpi.first.math.kinematics.SwerveModuleState[4];
		for (SwerveModule mod : mModules) {
			states[mod.moduleNumber()] = mod.getWpiState();
		}
		return states;
	}

	public Pose2d getPose() {
		return RobotState.getInstance().getLatestGlobalPose();
	}

	public void resetOdometry(Pose2d pose) {
		odometryReset = true;
		Pose2d wanted_pose = pose;
		mWheelTracker.resetPose(wanted_pose);
	}

	public void resetOdometry(edu.wpi.first.math.geometry.Pose2d pose) {
		resetOdometry(Util.to254Pose(pose));
	}

	public boolean readyForAuto() {
		return odometryReset;
	}

	public Rotation2d getHeading() {
		return mPigeon.getYaw();
	}
	public DriveMotionPlanner getMotionPlanner() {
		return mMotionPlanner;
	}

	public SwerveKinematicLimits getKinematicLimits() {
		return mKinematicLimits;
	}
	
	public static class SwerveInputs {
		// Inputs/Desired States
		double timestamp;
		ChassisSpeeds des_chassis_speeds = new ChassisSpeeds(0.0, 0.0, 0.0);
		Twist2d measured_velocity = Twist2d.identity();

		SwerveSetpoint setpoint = new SwerveSetpoint(new ChassisSpeeds(), new SwerveModuleState[] {
			new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()
		}); 
		Rotation2d heading = Rotation2d.identity();
		Rotation2d pitch = Rotation2d.identity();
		// Outputs
		SwerveModuleState[] des_module_states = new SwerveModuleState[] {
			new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()
		};
		SwerveModuleState[] uncapped_module_states = new SwerveModuleState[] {
			new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()
		};
		
		Twist2d predicted_velocity = Twist2d.identity();
		Translation2d translational_error = Translation2d.identity();
		Rotation2d heading_error = Rotation2d.identity();
		TimedState<Pose2dWithMotion> path_setpoint = new TimedState<Pose2dWithMotion>(Pose2dWithMotion.identity());
		Rotation2d heading_setpoint = new Rotation2d();
	}

	@Override
	public void outputTelemetry() {
		edu.wpi.first.math.kinematics.SwerveModuleState[] uncappedstates = new edu.wpi.first.math.kinematics.SwerveModuleState[4];
		edu.wpi.first.math.kinematics.SwerveModuleState[] states = new edu.wpi.first.math.kinematics.SwerveModuleState[4];
		for (SwerveModule mod : mModules) {
			states[mod.moduleNumber()] = mod.getWpiState();
			uncappedstates[mod.moduleNumber()] = mPeriodicIO.uncapped_module_states[mod.moduleNumber()].wpi();
		}
		Logger.recordOutput("Drive/Desired States", states);
		Logger.recordOutput("Drive/Uncapped States", uncappedstates);
		Logger.recordOutput("Drive/Current States", getWpiModuleStates());
		Logger.recordOutput("Drive/DesiredSpeed", mPeriodicIO.des_chassis_speeds.wpi());
		Logger.recordOutput("Drive/State",mControlState );
		Logger.recordOutput("Drive/Predicted Velocity", mPeriodicIO.predicted_velocity.wpi());
		Logger.recordOutput("Drive/Heading", mPeriodicIO.heading);
		Logger.recordOutput("Drive/Target Heading", mHeadingController.getTargetHeading());
		Logger.recordOutput("RobotState/Filtered Pose", RobotState.getInstance().getLatestGlobalPose().wpi());


		for (SwerveModule module : mModules) {
			module.outputTelemetry();

		}
	}

	public DriveControlState getControlState() {
		return mControlState;
	}

	@Override
	public void stop() {
		mPeriodicIO.des_chassis_speeds = new ChassisSpeeds();
		mControlState = DriveControlState.OPEN_LOOP;
	}

	@Override
	public boolean checkSystem() {
		return false;
	}
// 

}
