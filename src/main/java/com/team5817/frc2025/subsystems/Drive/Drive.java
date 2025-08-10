// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package com.team5817.frc2025.subsystems.Drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.swerve.ChassisSpeeds;
import com.team254.lib.util.SynchronousPIDF;
import com.team5817.frc2025.field.AlignmentPoint.AlignmentType;
import com.team5817.frc2025.generated.TunerConstants;
import com.team5817.lib.RobotMode;
import com.team5817.lib.RobotMode.Mode;
import com.team5817.lib.drivers.Subsystem;
import com.team5817.lib.motion.Trajectory;
import com.team5817.lib.swerve.DriveMotionPlanner;
import com.team5817.lib.swerve.GyroIO;
import com.team5817.lib.swerve.GyroIOInputsAutoLogged;
import com.team5817.lib.swerve.ModuleIO;
import com.team5817.lib.swerve.Module;
import com.team5817.lib.swerve.PhoenixOdometryThread;
import com.team5817.lib.swerve.SwerveHeadingController;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;

public class Drive extends Subsystem {
  // TunerConstants doesn't include these constants, so they are declared locally
  public static final double ODOMETRY_FREQUENCY = new CANBus(TunerConstants.DrivetrainConstants.CANBusName)
      .isNetworkFD() ? 250.0 : 100.0;
  public static final double DRIVE_BASE_RADIUS = Math.max(
      Math.max(
          Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
          Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
      Math.max(
          Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
          Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  // PathPlanner config constants
  private static final double ROBOT_MASS_KG = 74.088;
  private static final double ROBOT_MOI = 6.883;
  private static final double WHEEL_COF = 1.2;

  @Getter
  static final RobotConfig PP_CONFIG = new RobotConfig(
      ROBOT_MASS_KG,
      ROBOT_MOI,
      new ModuleConfig(
          TunerConstants.FrontLeft.WheelRadius,
          TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
          WHEEL_COF,
          DCMotor.getKrakenX60Foc(1)
              .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
          TunerConstants.FrontLeft.SlipCurrent,
          1),
      getModuleTranslations());

  public enum DriveControlState {
    FORCE_ORIENT,
    OPEN_LOOP,
    HEADING_SNAP,
    VELOCITY,
    PATH_FOLLOWING,
    AUTOALIGN
  }

  private DriveControlState mControlState = DriveControlState.OPEN_LOOP;
  private boolean mControlStateHasChanged = false;
  private double alignmentStartTimestamp = 0;
  private final DriveMotionPlanner mMotionPlanner;
  private final AutoAlignMotionPlanner mAutoAlignMotionPlanner;

  private final SwerveHeadingController mHeadingController;
  @Setter
  @Accessors(prefix = "m")
  private Rotation2d mTrackingAngle = Rotation2d.identity();
  @Setter
  @Accessors(prefix = "m")
  private boolean mOverrideHeading = false;

  @Setter
  @Accessors(prefix = "m")
  private AlignmentType mAlignment = AlignmentType.CORAL_SCORE;
  @Setter
  private boolean autoAlignFinishedOverride = false;

  public static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  // private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert = new Alert("Disconnected gyro, using kinematics as fallback.",
      AlertType.kError);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation.wpi(),
      lastModulePositions, new Pose2d().wpi());

      public Drive(
        GyroIO gyroIO,
        ModuleIO flModuleIO,
        ModuleIO frModuleIO,
        ModuleIO blModuleIO,
        ModuleIO brModuleIO,
        SynchronousPIDF stabilizePID,
        SynchronousPIDF snapPID
    ) {
        this.gyroIO = gyroIO;
    
        modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
        modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
        modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
        modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);
    
        mMotionPlanner = new DriveMotionPlanner();
    
        mHeadingController = new SwerveHeadingController(snapPID, stabilizePID);
    
        mAutoAlignMotionPlanner = new AutoAlignMotionPlanner( new SwerveHeadingController(snapPID, stabilizePID));
        mAutoAlignMotionPlanner.reset();
    
        // Usage reporting for swerve template
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);
    
        // Start odometry thread
        PhoenixOdometryThread.getInstance().start();

    // // Configure SysId
    // sysId = new SysIdRoutine(
    // new SysIdRoutine.Config(
    // null,
    // null,
    // null,
    // (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
    // new SysIdRoutine.Mechanism(
    // (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
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

  public void feedTeleopSetpoint(ChassisSpeeds speeds) {
    runVelocity(getTeleopSetpoint(speeds));
  }

  private boolean isStabilizing = true;

  private ChassisSpeeds getTeleopSetpoint(ChassisSpeeds speeds) {
    double timestamp = Timer.getTimestamp();

    boolean userSlowingDown = Math.abs(speeds.omegaRadiansPerSecond) < 0.15;
    boolean robotRotatingSlowly = Math.abs(gyroInputs.yawVelocityRadPerSec) < 0.3;
    
    if (!isStabilizing && userSlowingDown && robotRotatingSlowly) {
      mHeadingController.setStabilizeTarget(getHeading());
      isStabilizing = true;
    }
    
    if (!userSlowingDown) {
      isStabilizing = false;
    }
    
    double omega = isStabilizing
        ? mHeadingController.update(getHeading(), timestamp)
        : 0.0;
    
    switch (mControlState) {
      case PATH_FOLLOWING:
        if (Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) <= TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.1) {
        mControlStateHasChanged = false;
          return new ChassisSpeeds();
        }
        break;
    
      case AUTOALIGN:
        if (mControlStateHasChanged) {
          alignmentStartTimestamp = timestamp;
        }
        boolean movingFast = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) > TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.1;
        boolean timeoutPassed = timestamp - alignmentStartTimestamp > 0.5;
        if (movingFast && timeoutPassed) {
          mControlStateHasChanged = false;
          setControlState(DriveControlState.OPEN_LOOP);
          return speeds;
        }
        ChassisSpeeds alignSpeeds = mAutoAlignMotionPlanner.updateAutoAlign(timestamp, getPose());
        mControlStateHasChanged = false;
        return alignSpeeds != null ? alignSpeeds : new ChassisSpeeds();
    
      case OPEN_LOOP:
      case HEADING_SNAP:
        if (Math.abs(speeds.omegaRadiansPerSecond) > 0.1) {
          omega = speeds.omegaRadiansPerSecond;
        }
        if (Math.abs(omega) < 0.05) {
          omega = 0.0;
        }
        mControlStateHasChanged = false;
        return new ChassisSpeeds(
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            omega
        );
    
      default:
        setControlState(DriveControlState.OPEN_LOOP);
        break;
    }
    
    mControlStateHasChanged = false;
    return speeds;
    
  }

  public void autoAlign(AlignmentType type) {
    setAlignment(type);
    alignDrive(findTargetPoint(), mAlignment.tolerance);
  }

  /**
   * Initiates auto alignment with the specified alignment type.
   *
   * @param type The alignment type.
   */
  public Pose2d findTargetPoint() {
    return AutoAlignPointSelector.chooseTargetPoint(getPose(), mAlignment);
  }

  public void alignDrive(Pose2d targetPoint, Pose2d tolerance) {
    autoAlignFinishedOverride = false;
    if (targetPoint == null) {
      System.out.println("Invalid target point");
      return;
    }
    mAutoAlignMotionPlanner.setTargetPoint(targetPoint, tolerance);

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
    setControlState(DriveControlState.PATH_FOLLOWING);
  }

  /**
   * Checks if auto alignment is complete.
   *
   * @return True if auto alignment is complete, false otherwise.
   */
  public boolean getAutoAlignComplete() {
    if (autoAlignFinishedOverride)
      return true;
    return mAutoAlignMotionPlanner.getAutoAlignComplete();
  }

  /**
   * Checks if auto alignment is complete.
   *
   * @return True if auto alignment is complete, false otherwise.
   */
  public Translation2d getAutoAlignError() {
    mAutoAlignMotionPlanner.updateAutoAlign(Timer.getTimestamp(),
        getPose()
            .withRotation(getHeading()));
    return mAutoAlignMotionPlanner.getAutoAlignError();
  }

  /**
   * Updates the path follower.
   */
  public void updatePathFollower() {
    final double now = Timer.getTimestamp();
    ChassisSpeeds output = mMotionPlanner.update(now, getPose());
    if (output != null) {
      runVelocity(output);
    }
  }

  /**
   * Checks if the trajectory is finished.
   *
   * @return True if the trajectory is finished, false otherwise.
   */
  public boolean isTrajectoryFinished() {
    return mMotionPlanner.isPathFinished();
  }

  public void updateAuto(){
    switch (mControlState) {
      case PATH_FOLLOWING:
        updatePathFollower();
        break;

      case AUTOALIGN:
        runVelocity(mAutoAlignMotionPlanner.updateAutoAlign(Timer.getTimestamp(), getPose()));
        break;

      default:
        System.out.println("Unsupported Drive Auto ControlState");
        break;
    }
  }

  
  @Override
  public void readPeriodicInputs() {
    if(DriverStation.isAutonomous())
      updateAuto();
  
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);

    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Update odometry
    double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] = new SwerveModulePosition(
            modulePositions[moduleIndex].distanceMeters
                - lastModulePositions[moduleIndex].distanceMeters,
            modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = new Twist2d(kinematics.toTwist2d(moduleDeltas));
        rawGyroRotation = rawGyroRotation.add(Rotation2d.fromRadians(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation.wpi(), modulePositions);
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && RobotMode.mode != Mode.SIM);
  }

  @Override
  public void outputTelemetry() {
    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("Drive/SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("Drive/SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    Logger.recordOutput("Drive/State", mControlState);
    Logger.recordOutput("Drive/SwerveStates/Measured", getModuleStates());
    Logger.recordOutput("Drive/SwerveChassisSpeeds/Measured", getChassisSpeeds().wpi());

    Logger.recordOutput("AutoAlign/mAlignment", mAlignment);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds.wpi());
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("Drive/SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("Drive/SwerveChassisSpeeds/Setpoints", discreteSpeeds.wpi());

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("Drive/SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement.
   * The modules will
   * return to their normal orientations the next time a nonzero velocity is
   * requested.
   */
  public void stopWithX() {
    edu.wpi.first.math.geometry.Rotation2d[] headings = new edu.wpi.first.math.geometry.Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  // /** Returns a command to run a quasistatic test in the specified direction.
  // */
  // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  // return run(() -> runCharacterization(0.0))
  // .withTimeout(1.0)
  // .andThen(sysId.quasistatic(direction));
  // }

  // /** Returns a command to run a dynamic test in the specified direction. */
  // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  // return run(() ->
  // runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  // }

  /**
   * Returns the module states (turn angles and drive velocities) for all of the
   * modules.
   */
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /**
   * Returns the module positions (turn angles and drive positions) for all of the
   * modules.
   */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  private ChassisSpeeds getChassisSpeeds() {
    return new ChassisSpeeds(kinematics.toChassisSpeeds(getModuleStates()));
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /**
   * Returns the average velocity of the modules in rotations/sec (Phoenix native
   * units).
   */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return new Pose2d(poseEstimator.getEstimatedPosition());
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  public void zeroGyro(double newHeading) {
    gyroIO.resetYaw(Rotation2d.fromDegrees(newHeading));
  }

  public void zeroGyro() {
    zeroGyro(0);
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation.wpi(), getModulePositions(), pose.wpi());
  }
  public void simResetWorldPose(Pose2d newPose){}

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters.wpi(), timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  /** Returns an array of module translations. */
  public static edu.wpi.first.math.geometry.Translation2d[] getModuleTranslations() {
    return new edu.wpi.first.math.geometry.Translation2d[] {
        new edu.wpi.first.math.geometry.Translation2d(TunerConstants.FrontLeft.LocationX,
            TunerConstants.FrontLeft.LocationY),
        new edu.wpi.first.math.geometry.Translation2d(TunerConstants.FrontRight.LocationX,
            TunerConstants.FrontRight.LocationY),
        new edu.wpi.first.math.geometry.Translation2d(TunerConstants.BackLeft.LocationX,
            TunerConstants.BackLeft.LocationY),
        new edu.wpi.first.math.geometry.Translation2d(TunerConstants.BackRight.LocationX,
            TunerConstants.BackRight.LocationY)
    };
  }
}
