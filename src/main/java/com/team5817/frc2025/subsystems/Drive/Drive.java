package com.team5817.frc2025.subsystems.Drive;

import com.team5817.frc2025.RobotConstants;
import com.team5817.frc2025.RobotState;
import com.team5817.frc2025.field.AlignmentPoint;
import com.team5817.frc2025.field.AlignmentPoint.AlignmentType;
import com.team5817.frc2025.loops.ILooper;
import com.team5817.frc2025.loops.Loop;
import com.team5817.frc2025.subsystems.Cancoders;
import com.team5817.frc2025.subsystems.WheelTracker;
import com.team5817.frc2025.subsystems.Drive.SwerveConstants.Mod0;
import com.team5817.frc2025.subsystems.Drive.SwerveConstants.Mod1;
import com.team5817.frc2025.subsystems.Drive.SwerveConstants.Mod2;
import com.team5817.frc2025.subsystems.Drive.SwerveConstants.Mod3;
import com.team5817.lib.RobotMode;
import com.team5817.lib.Util;
import com.team5817.lib.drivers.Pigeon;
import com.team5817.lib.drivers.Subsystem;
import com.team5817.lib.motion.PPPathPointState;
import com.team5817.lib.motion.Trajectory;
import com.team5817.lib.swerve.DriveMotionPlanner;

import com.team5817.lib.swerve.SwerveHeadingController;
import com.team5817.lib.swerve.SwerveModule;
import com.team5817.lib.swerve.SwerveModulePosition;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.swerve.ChassisSpeeds;
import com.team254.lib.swerve.SwerveKinematicLimits;
import com.team254.lib.swerve.SwerveModuleState;
import com.team254.lib.swerve.SwerveSetpoint;
import com.team254.lib.swerve.SwerveSetpointGenerator;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Pound;
import static edu.wpi.first.units.Units.Volt;

import java.util.ArrayList;
import java.util.List;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
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
  private Pigeon mPigeon;
  public SwerveModule[] mModules;

  private SwerveInputs mPeriodicIO = new SwerveInputs();
  private DriveControlState mControlState = DriveControlState.OPEN_LOOP;

  private SwerveSetpointGenerator mSetpointGenerator;

  private boolean odometryReset = false;

  private final DriveMotionPlanner mMotionPlanner;
  private final AutoAlignMotionPlanner mAutoAlignMotionPlanner = new AutoAlignMotionPlanner();
  private final SwerveHeadingController mHeadingController;

  private boolean mControlStateHasChanged = false;
  private boolean mOverrideHeading = false;

  private double alignmentStartTimestamp = 0;

  private Rotation2d mTrackingAngle = Rotation2d.identity();

  private SwerveKinematicLimits mKinematicLimits = SwerveConstants.kSwerveKinematicLimits;
  private SwerveKinematicLimits driverKinematicLimits = mKinematicLimits;
  private SwerveKinematicLimits mUncappedKinematicLimits = SwerveConstants.kSwerveUncappedKinematicLimits;

  private static AlignmentType mAlignment = AlignmentType.CORAL_SCORE;
  private static Drive mInstance;

  public static Drive getInstance() {
    if (mInstance == null) {
      mInstance = new Drive();
    }
    return mInstance;
  }

  public static final DriveTrainSimulationConfig mapleSimConfig = DriveTrainSimulationConfig.Default()
      .withRobotMass(Pound.of(115))
      .withTrackLengthTrackWidth(Inches.of(35), Inches.of(35))
      .withGyro(COTS.ofPigeon2())
      .withSwerveModule(new SwerveModuleSimulationConfig(
          DCMotor.getKrakenX60Foc(1),
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

    Pigeon.registerGyroSim(driveSimulation);
    mPigeon = Pigeon.getInstance();
    mPigeon.setYaw(0.0);
    mWheelTracker = new WheelTracker(mModules);
    mSetpointGenerator = new SwerveSetpointGenerator(SwerveConstants.kKinematics);
    mAutoAlignMotionPlanner.reset();

  }

  /**
   * Sets the control state of the drive system.
   *
   * @param newState The new control state to set.
   */
  public void setControlState(DriveControlState newState) {
    if (newState != mControlState) {
      mControlState = newState;
      mControlStateHasChanged = true;
    }
  }

  /**
   * Updates drivetrain with latest desired speeds from the joystick, and sets
   * DriveControlState appropriately.
   *
   * @param speeds ChassisSpeeds object derived from joystick input
   */

  public void feedTeleopSetpoint(ChassisSpeeds speeds) {

    double omega = mHeadingController.update(mPeriodicIO.heading, Timer.getTimestamp());

    if (mControlState != DriveControlState.HEADING_CONTROL
        && Math.abs(mPeriodicIO.setpoint.mChassisSpeeds.omegaRadiansPerSecond - omega) > .2) {
      mHeadingController.setStabilizeTarget(mPigeon.getYaw());
    }

    if (mControlState == DriveControlState.PATH_FOLLOWING) {
      if (Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) > mKinematicLimits.kMaxDriveVelocity
          * 0.1) {
        // setControlState(DriveControlState.OPEN_LOOP);

      } else {

        mControlStateHasChanged = false;
        return;
      }
    }
    if (mControlState == DriveControlState.AUTOALIGN) {
      if (mControlStateHasChanged)
        alignmentStartTimestamp = Timer.getTimestamp();
      if (Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) > mKinematicLimits.kMaxDriveVelocity
          * 0.1 && Timer.getTimestamp() - alignmentStartTimestamp > .5) {
        mPeriodicIO.des_chassis_speeds = speeds;
        mControlStateHasChanged = false;
        return;
      } else {
        ChassisSpeeds speed = mAutoAlignMotionPlanner.updateAutoAlign(mPeriodicIO.timestamp,
            RobotState.getInstance().getGlobalKalmanPose(mPeriodicIO.timestamp)
                .withRotation(mPeriodicIO.heading));
        if (speed != null) {
          mPeriodicIO.des_chassis_speeds = speed;
        }
        mControlStateHasChanged = false;
        return;
      }
    }

    if (mControlState == DriveControlState.OPEN_LOOP || mControlState == DriveControlState.HEADING_CONTROL) {
      mKinematicLimits = driverKinematicLimits;
      double x = speeds.vxMetersPerSecond;
      double y = speeds.vyMetersPerSecond;

      if (Math.abs(speeds.omegaRadiansPerSecond) > .1) {
        omega = speeds.omegaRadiansPerSecond;

      }

      if (Math.abs(omega) < .05) {
        omega = 0;
      }

      mPeriodicIO.des_chassis_speeds = new ChassisSpeeds(x, y, omega);

      mControlStateHasChanged = false;
      return;

    } else if (mControlState != DriveControlState.OPEN_LOOP) {
      setControlState(DriveControlState.OPEN_LOOP);

    }

    mControlStateHasChanged = false;
    mPeriodicIO.des_chassis_speeds = speeds;
  }

  /**
   * Sets the open loop control state with the given speeds.
   *
   * @param speeds The desired chassis speeds.
   */
  public void setOpenLoop(ChassisSpeeds speeds) {
    mPeriodicIO.des_chassis_speeds = speeds;
    if (mControlState != DriveControlState.OPEN_LOOP) {
      mControlState = DriveControlState.OPEN_LOOP;
    }
  }

  /**
   * Sets the velocity control state with the given speeds.
   *
   * @param speeds The desired chassis speeds.
   */
  public void setVelocity(ChassisSpeeds speeds) {
    mPeriodicIO.des_chassis_speeds = speeds;
    if (mControlState != DriveControlState.VELOCITY) {
      mControlState = DriveControlState.VELOCITY;
    }
  }

  public void setDriverKinematicLimits(SwerveKinematicLimits limits) {
    driverKinematicLimits = limits;
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

  /**
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

  /**
   * Sets the alignment type for auto alignment.
   *
   * @param alignment The alignment type.
   */
  public void setAlignment(AlignmentType alignment) {
    mAlignment = alignment;
  }

  /**
   * Enable/disables vision heading control.
   *
   * @param value Whether or not to override rotation joystick with vision target.
   */
  public void overrideHeading(boolean value) {
    mOverrideHeading = value;
  }

  /**
   * Updates needed angle to track a goal.
   *
   * @param angle Sets the wanted robot heading to track a goal.
   */
  public void feedTrackingSetpoint(Rotation2d angle) {
    mTrackingAngle = angle;
  }

  /**
   * Stops modules in place.
   */
  public void stopModules() {
    List<Rotation2d> orientations = new ArrayList<>();
    for (SwerveModuleState SwerveModuleState : mPeriodicIO.des_module_states) {
      orientations.add(SwerveModuleState.angle);
    }
    orientModules(orientations);
  }

  /**
   * Orients modules to the angles provided.
   * 
   * @param orientations Rotation2d of target angles, indexed by module number.
   */
  public void orientModules(List<Rotation2d> orientations) {
    if (mControlState != DriveControlState.FORCE_ORIENT) {
      mControlState = DriveControlState.FORCE_ORIENT;
    }
    for (int i = 0; i < mModules.length; ++i) {
      mPeriodicIO.des_module_states[i] = new SwerveModuleState(0.0, orientations.get(i));
    }
  }

  boolean autoAlignFinishedOverrride = false;

  public void autoAlign(AlignmentType type) {
    setAlignment(type);
    alignDrive(findTargetPoint());
  }

  /**
   * Initiates auto alignment with the specified alignment type.
   *
   * @param type The alignment type.
   */
  public Pose2d findTargetPoint() {
    return AutoAlignPointSelector.chooseTargetPoint(getPose(), mAlignment);
  }

  private void alignDrive(Pose2d targetPoint) {
    autoAlignFinishedOverrride = false;
    if (targetPoint == null) {
      return;
    }
    mKinematicLimits = SwerveConstants.kSwerveKinematicLimits;
    mAutoAlignMotionPlanner.setTargetPoint(targetPoint, mAlignment.tolerance);
    if (mControlState != DriveControlState.AUTOALIGN) {
      mAutoAlignMotionPlanner.reset();
      setControlState(DriveControlState.AUTOALIGN);
    }
  }

  /**
   * Sets the trajectory for the motion planner.
   *
   * @param trajectory The trajectory to follow.
   */
  public void setTrajectory(Trajectory trajectory, double timeout) {
    mMotionPlanner.reset();
    mMotionPlanner.setTrajectory(trajectory.get(), timeout);
    mKinematicLimits = SwerveConstants.kSwerveUncappedKinematicLimits;
    setControlState(DriveControlState.PATH_FOLLOWING);
  }

  /**
   * Checks if auto alignment is complete.
   *
   * @return True if auto alignment is complete, false otherwise.
   */
  public boolean getAutoAlignComplete() {
    if (autoAlignFinishedOverrride)
      return true;
    return mAutoAlignMotionPlanner.getAutoAlignComplete();
  }

  /**
   * Checks if auto alignment is complete.
   *
   * @return True if auto alignment is complete, false otherwise.
   */
  public Translation2d getAutoAlignError() {
    mAutoAlignMotionPlanner.setTargetPoint(findTargetPoint(), mAlignment.tolerance);
    mAutoAlignMotionPlanner.updateAutoAlign(mPeriodicIO.timestamp,
        RobotState.getInstance().getGlobalKalmanPose(mPeriodicIO.timestamp)
            .withRotation(mPeriodicIO.heading));
    return mAutoAlignMotionPlanner.getAutoAlignError();
  }

  /**
   * Overrides the auto alignment completion status.
   *
   * @param override True to override, false otherwise.
   */
  public void autoAlignFinishedOverrride(boolean override) {
    autoAlignFinishedOverrride = override;
  }

  /**
   * Updates the path follower.
   */
  public void updatePathFollower() {
    final double now = Timer.getTimestamp();
    ChassisSpeeds output = mMotionPlanner.update(now, getPose());
    if (output != null) {
      mPeriodicIO.des_chassis_speeds = output;
    }
    mPeriodicIO.translational_error = mMotionPlanner.getTranslationalError();
    mPeriodicIO.heading_error = mMotionPlanner.getHeadingError();
    mPeriodicIO.path_setpoint = mMotionPlanner.getSetpoint();
  }

  /**
   * Checks if the trajectory is finished.
   *
   * @return True if the trajectory is finished, false otherwise.
   */
  public boolean isTrajectoryFinished() {
    return mMotionPlanner.isPathFinished();
  }

  @Override
  public void registerEnabledLoops(ILooper enabledLooper) {
    enabledLooper.register(new Loop() {
      @Override
      public void onStart(double timestamp) {
        mPeriodicIO.des_chassis_speeds = new ChassisSpeeds();
        mControlState = DriveControlState.OPEN_LOOP;
      }

      @Override
      public void onLoop(double timestamp) {
        switch (mControlState) {
          case PATH_FOLLOWING:
            updatePathFollower();
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
            .addOdomObservations(
                timestamp,
                mWheelTracker.getRobotPose(),
                mPeriodicIO.measured_velocity,
                mPeriodicIO.predicted_velocity);
        mWheelTracker.readPeriodicInputs();

      }
    });
  }

  @Override
  public void readPeriodicInputs() {
    SwerveModuleState[] module_states = new SwerveModuleState[4];
    if (RobotMode.isSim()) {
      for (int i = 0; i < mModules.length; i++) {
        module_states[i] = new SwerveModuleState(driveSimulation.getModules()[i].getCurrentState());
      }
    } else {
      for (SwerveModule swerveModule : mModules) {
        swerveModule.readPeriodicInputs();
      }
      module_states = getModuleStates();
    }
    mPeriodicIO.pitch = mPigeon.getPitch();
    mPeriodicIO.heading = mPigeon.getYaw();
    mPeriodicIO.timestamp = Timer.getFPGATimestamp();
    Twist2d twist_vel = com.team5817.frc2025.subsystems.Drive.SwerveConstants.kKinematics
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
   * Updates the wanted setpoint, including whether heading should
   * be overridden to the tracking angle. Also includes
   * updates for Path Following.
   */
  private void updateSetpoint() {
    if (mControlState == DriveControlState.FORCE_ORIENT)
      return;

    Pose2d robot_pose_vel = new Pose2d(
        mPeriodicIO.des_chassis_speeds.vxMetersPerSecond * RobotConstants.kLooperDt * 4.0,
        mPeriodicIO.des_chassis_speeds.vyMetersPerSecond * RobotConstants.kLooperDt * 4.0,
        Rotation2d.fromRadians(
            mPeriodicIO.des_chassis_speeds.omegaRadiansPerSecond * RobotConstants.kLooperDt * 4.0));

    Twist2d twist_vel = Pose2d.log(robot_pose_vel).scaled(1.0 / (4.0 * RobotConstants.kLooperDt));

    ChassisSpeeds wanted_speeds;
    if (mOverrideHeading) {
      stabilizeHeading(mTrackingAngle);
      double new_omega = mHeadingController.update(mPigeon.getYaw(), Timer.getTimestamp());
      ChassisSpeeds speeds = new ChassisSpeeds(twist_vel.dx, twist_vel.dy, new_omega);
      wanted_speeds = speeds;
    } else {
      wanted_speeds = new ChassisSpeeds(twist_vel.dx, twist_vel.dy, twist_vel.dtheta);
    }
    wanted_speeds = ChassisSpeeds.discretize(wanted_speeds, RobotConstants.kLooperDt);

    mPeriodicIO.setpoint = mSetpointGenerator.generateSetpoint(mKinematicLimits, mPeriodicIO.setpoint,
        wanted_speeds, RobotConstants.kLooperDt);
    var uncapped_setpoint = mSetpointGenerator.generateSetpoint(mUncappedKinematicLimits, mPeriodicIO.setpoint,
        wanted_speeds, RobotConstants.kLooperDt);
    mPeriodicIO.predicted_velocity = wanted_speeds.toTwist2d();
    // Pose2d.log(Pose2d.exp(wanted_speeds.toTwist2d()).rotateBy(getHeading()));

    mPeriodicIO.uncapped_module_states = uncapped_setpoint.mModuleStates;
    mPeriodicIO.des_module_states = mPeriodicIO.setpoint.mModuleStates;

  }

  /**
   * Resets the modules to their absolute positions.
   */
  public void resetModulesToAbsolute() {
    for (SwerveModule module : mModules) {
      module.resetToAbsolute();
    }
  }

  /**
   * Zeros the gyro.
   */
  public void zeroGyro() {
    zeroGyro(0.0);
  }

  /**
   * Zeros the gyro to a specific angle.
   *
   * @param reset_deg The angle to reset the gyro to.
   */
  public void zeroGyro(double reset_deg) {
    mPigeon.setYaw(reset_deg);
  }

  /**
   * Configures if module drive motors should brake when commanded neutral output.
   * 
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
      } else if (mControlState == DriveControlState.PATH_FOLLOWING
          || mControlState == DriveControlState.VELOCITY
          || mControlState == DriveControlState.FORCE_ORIENT
          || mControlState == DriveControlState.AUTOALIGN) {
        mModules[i].setVelocity(mPeriodicIO.des_module_states[i]);
      }
    }
    if (RobotMode.isSim()) {
      driveSimulation.setRobotSpeeds(ChassisSpeeds
          .fromFieldRelativeSpeeds(mPeriodicIO.setpoint.mChassisSpeeds, mPeriodicIO.heading.inverse()).wpi());
    } else {
      for (SwerveModule swerveModule : mModules) {
        swerveModule.writePeriodicOutputs();
      }
    }

  }

  /**
   * Gets the states of the swerve modules.
   *
   * @return An array of SwerveModuleState objects representing the states of the
   *         modules.
   */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mModules) {
      states[mod.moduleNumber()] = mod.getState();
    }
    return states;
  }

  /**
   * Gets the positions of the swerve modules.
   *
   * @return An array of SwerveModulePosition objects representing the positions
   *         of the modules.
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (SwerveModule mod : mModules) {
      states[mod.moduleNumber()] = mod.getPosition();
    }
    return states;
  }

  /**
   * Gets the positions of the swerve modules in WPILib format.
   *
   * @return An array of WPILib SwerveModulePosition objects representing the
   *         positions of the modules.
   */
  public edu.wpi.first.math.kinematics.SwerveModulePosition[] getWpiModulePositions() {
    edu.wpi.first.math.kinematics.SwerveModulePosition[] states = new edu.wpi.first.math.kinematics.SwerveModulePosition[4];
    for (SwerveModule mod : mModules) {
      states[mod.moduleNumber()] = mod.getWpiPosition();
    }
    return states;
  }

  /**
   * Gets the states of the swerve modules in WPILib format.
   *
   * @return An array of WPILib SwerveModuleState objects representing the states
   *         of the modules.
   */
  public edu.wpi.first.math.kinematics.SwerveModuleState[] getWpiModuleStates() {
    edu.wpi.first.math.kinematics.SwerveModuleState[] states = new edu.wpi.first.math.kinematics.SwerveModuleState[4];
    for (SwerveModule mod : mModules) {
      states[mod.moduleNumber()] = mod.getWpiState();
    }
    return states;
  }

  /**
   * Gets the pose of the robot in WPILib format.
   *
   * @return The pose of the robot.
   */
  public edu.wpi.first.math.geometry.Pose2d getWpiPose() {
    return getPose().wpi();
  }

  /**
   * Gets the pose of the robot.
   *
   * @return The pose of the robot.
   */
  public Pose2d getPose() {
    return RobotState.getInstance().getLatestGlobalKalmanPose();
  }

  /**
   * Resets the odometry to a specific pose.
   *
   * @param pose The pose to reset the odometry to.
   */
  public void resetOdometry(Pose2d pose) {
    odometryReset = true;
    Pose2d wanted_pose = pose;
    mWheelTracker.resetPose(wanted_pose);
  }

  /**
   * Resets the odometry to a specific pose in WPILib format.
   *
   * @param pose The pose to reset the odometry to.
   */
  public void resetOdometry(edu.wpi.first.math.geometry.Pose2d pose) {
    resetOdometry(Util.to254Pose(pose));
  }

  /**
   * Checks if the robot is ready for autonomous mode.
   *
   * @return True if the robot is ready for autonomous mode, false otherwise.
   */
  public boolean readyForAuto() {
    return odometryReset;
  }

  /**
   * Gets the heading of the robot.
   *
   * @return The heading of the robot.
   */
  public Rotation2d getHeading() {
    return mPigeon.getYaw();
  }

  /**
   * Gets the motion planner.
   *
   * @return The motion planner.
   */
  public DriveMotionPlanner getMotionPlanner() {
    return mMotionPlanner;
  }

  /**
   * Gets the kinematic limits.
   *
   * @return The kinematic limits.
   */
  public SwerveKinematicLimits getKinematicLimits() {
    return mKinematicLimits;
  }

  public static AlignmentPoint getAlignment() {
    return AutoAlignPointSelector.a;
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
    PPPathPointState path_setpoint = new PPPathPointState();
    Rotation2d heading_setpoint = new Rotation2d();
  }

  Field2d mField2d = new Field2d();

  @Override
  public void outputTelemetry() {

    PPLibTelemetry.setCurrentPose(getWpiPose());

    if (mMotionPlanner.getPath() != null) {
      var speeds = RobotState.getInstance().getSmoothedVelocity();
      var feedforwards = mMotionPlanner.getSetpoint();
      PPLibTelemetry.setCurrentPath(mMotionPlanner.getPath());
      PPLibTelemetry.setTargetPose(feedforwards.getPose().wpi());
      PPLibTelemetry.setVelocities(speeds.norm(), Math.hypot(feedforwards.getXVel(), feedforwards.getYVel()),
          speeds.dtheta, feedforwards.getThetaVel());
    }
    edu.wpi.first.math.kinematics.SwerveModuleState[] currentStates = new edu.wpi.first.math.kinematics.SwerveModuleState[4];
    edu.wpi.first.math.kinematics.SwerveModuleState[] states = new edu.wpi.first.math.kinematics.SwerveModuleState[4];
    for (SwerveModule mod : mModules) {
      states[mod.moduleNumber()] = mod.getWpiState();
      currentStates[mod.moduleNumber()] = mPeriodicIO.des_module_states[mod.moduleNumber()].wpi();
    }

    Logger.recordOutput("Drive/desired States", currentStates);
    Logger.recordOutput("Drive/Current States", getWpiModuleStates());
    Logger.recordOutput("Drive/DesiredSpeed", mPeriodicIO.des_chassis_speeds.wpi());
    Logger.recordOutput("Drive/State", mControlState);
    Logger.recordOutput("Drive/Predicted Velocity", mPeriodicIO.predicted_velocity.wpi());
    Logger.recordOutput("Drive/Heading", mPeriodicIO.heading);
    Logger.recordOutput("Drive/Target Heading", mHeadingController.getTargetHeading());
    Logger.recordOutput("RobotState/Filtered Pose", new Pose3d(
        RobotState.getInstance().getLatestGlobalKalmanPose().wpi().getX(),
        RobotState.getInstance().getLatestGlobalKalmanPose().wpi().getY(), 0.0,
        new Rotation3d(0, 0, RobotState.getInstance().getLatestGlobalKalmanPose().wpi().getRotation().getRadians())));
    Logger.recordOutput("RobotState/Odom Pose",
        new Pose3d(RobotState.getInstance().getLatestGlobalKalmanPose().wpi().getX(),
            RobotState.getInstance().getLatestGlobalKalmanPose().wpi().getY(), 0.0, new Rotation3d(0, 0,
                RobotState.getInstance().getLatestPoseFromOdom().getValue().wpi().getRotation().getRadians())));
    // Logger.recordOutput("RobotState/Specialized Pose",
    // RobotState.getInstance().getLatestSpecializedKalmanPose().wpi());

    Logger.recordOutput("Drive/Control State Changed", mControlStateHasChanged);

    // elastic
    mField2d.setRobotPose(RobotState.getInstance().getLatestGlobalKalmanPose().wpi());
    SmartDashboard.putData("Elastic/Pose", mField2d);

    for (SwerveModule module : mModules) {
      module.outputTelemetry();

    }
  }

  /**
   * Gets the current control state.
   *
   * @return The current control state.
   */
  public DriveControlState getControlState() {
    return mControlState;
  }

  @Override
  public void stop() {
    mPeriodicIO.des_chassis_speeds = new ChassisSpeeds();
  }

  @Override
  public boolean checkSystem() {
    return false;
  }
  //

}
