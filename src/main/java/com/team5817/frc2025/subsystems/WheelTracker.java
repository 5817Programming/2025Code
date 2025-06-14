package com.team5817.frc2025.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.team5817.frc2025.subsystems.Drive.Drive;
import com.team5817.lib.RobotMode;
import com.team5817.lib.drivers.Pigeon;
import com.team5817.lib.drivers.Subsystem;
import com.team5817.lib.swerve.SwerveModule;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.swerve.ChassisSpeeds;

import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

/**
 * The WheelTracker class is responsible for tracking the position and velocity
 * of the robot
 * using the swerve modules and a Pigeon IMU.
 */
public class WheelTracker extends Subsystem {
  private final Pigeon mPigeon = Pigeon.getInstance();
  private final SwerveModule[] mModules;

  private WheelProperties[] wheels = new WheelProperties[4];

  private double robotHeading;

  private double mTimestamp;

  private BaseStatusSignal[] mAllSignals;

  private OdometryThread mOdometryThread;
  private WheelTrackerInputsAutoLogged inputs = new WheelTrackerInputsAutoLogged();

  /**
   * Constructs a WheelTracker with the given swerve modules.
   *
   * @param modules The swerve modules used for odometry.
   * @throws IllegalArgumentException if the number of modules is not 4.
   */
  public WheelTracker(SwerveModule[] modules) {
    if (modules.length != 4) {
      throw new IllegalArgumentException("Odometry needs 4 modules to run");
    }

    mModules = modules;

    for (int i = 0; i < wheels.length; i++) {
      WheelProperties w = new WheelProperties();
      Translation2d robotToWheel = new Translation2d(
          com.team5817.frc2025.subsystems.Drive.SwerveConstants.swerveModuleLocations[i].x(),
          com.team5817.frc2025.subsystems.Drive.SwerveConstants.swerveModuleLocations[i].y());
      w.startingPosition = robotToWheel;
      wheels[i] = w;
    }

    resetModulePoses();

    mAllSignals = new BaseStatusSignal[(4 * 4) + 2];
    for (int i = 0; i < 4; ++i) {
      var signals = mModules[i].getUsedStatusSignals();
      mAllSignals[(i * 4) + 0] = signals[0];
      mAllSignals[(i * 4) + 1] = signals[1];
      mAllSignals[(i * 4) + 2] = signals[2];
      mAllSignals[(i * 4) + 3] = signals[3];
    }
    mAllSignals[mAllSignals.length - 2] = mPigeon.getYawStatusSignal();
    mAllSignals[mAllSignals.length - 1] = mPigeon.getRateStatusSignal();

    for (BaseStatusSignal sig : mAllSignals) {
      sig.setUpdateFrequency(250);
    }
    mOdometryThread = new OdometryThread();
    mOdometryThread.setDaemon(true);
    mOdometryThread.start();
  }

  @Override
  public void readPeriodicInputs() {
    Logger.processInputs("Wheel Tracker", inputs);
  }

  private class OdometryThread extends Thread {
    @Override
    public void run() {
      while (true) {
        try {
          BaseStatusSignal.waitForAll(1.0, mAllSignals);

          for (SwerveModule m : mModules) {
            m.refreshSignals(); // No downside to refreshing io reads from multiple threads
          }

          robotHeading = mPigeon.getYaw().getRadians();
          updateRobotPose(Timer.getTimestamp());
        } catch (Exception e) {
          e.printStackTrace();
        }
      }
    }

  }

  private Pose2d last_velocity_sample = new Pose2d();
  private double last_sample_timestamp = 0.0;

  /**
   * Updates the robot's pose based on the current timestamp.
   *
   * @param timestamp The current timestamp.
   */
  private void updateRobotPose(double timestamp) {
    double x = 0.0;
    double y = 0.0;
    Rotation2d heading = Rotation2d.fromRadians(robotHeading);

    double avg_delta = 0.0;
    double[] deltas = new double[4];
    for (int i = 0; i < mModules.length; i++) {
      SwerveModule m = mModules[i];
      WheelProperties w = wheels[i];
      updateWheelOdometry(m, w);
      double delta = w.estimatedRobotPose
          .getTranslation()
          .translateBy(inputs.pose.getTranslation().inverse())
          .norm();
      deltas[i] = delta;
      avg_delta += delta;
    }
    avg_delta /= 4;

    int min__dev_idx = 0;
    double min_dev = Double.MAX_VALUE;
    List<WheelProperties> accurateModules = new ArrayList<>();
    for (int i = 0; i < deltas.length; i++) {
      WheelProperties w = wheels[i];
      double dev = Math.abs(deltas[i] - avg_delta);
      if (dev < min_dev) {
        min_dev = dev;
        min__dev_idx = i;
      }
      if (dev <= 0.01) {
        accurateModules.add(w);
      }
    }

    if (accurateModules.isEmpty()) {
      accurateModules.add(wheels[min__dev_idx]);
    }

    int n = accurateModules.size();

    for (WheelProperties w : accurateModules) {
      x += w.estimatedRobotPose.getTranslation().x();
      y += w.estimatedRobotPose.getTranslation().y();
    }
    final Pose2d new_pose = new Pose2d(new Translation2d(x / n, y / n), heading);

    // Velocity calcs
    double sample_window = timestamp - last_sample_timestamp;
    if (sample_window > 0.02) {
      final Translation2d translation = (new_pose.transformBy(last_velocity_sample.inverse()).getTranslation());
      inputs.velocity = translation.scale(1.0 / sample_window);
      last_sample_timestamp = timestamp;
      last_velocity_sample = new_pose;
    }
    inputs.pose = new_pose;
    if (RobotMode.isSim()) {
      inputs.pose = new Pose2d(Drive.driveSimulation.getSimulatedDriveTrainPose());
      inputs.velocity = new ChassisSpeeds(
          Drive.driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative()).getTranslation();
    }

    resetModulePoses(inputs.pose);
  }

  @AutoLog
  public static class WheelTrackerInputs {
    Pose2d pose = new Pose2d();
    Translation2d velocity = new Translation2d();
  }

  /**
   * Updates the odometry for a single wheel module.
   *
   * @param module The swerve module.
   * @param props  The properties of the wheel.
   */
  private void updateWheelOdometry(SwerveModule module, WheelProperties props) {
    double currentEncDistance = module.getDriveDistanceMeters();
    double deltaEncDistance = currentEncDistance - props.previousEncDistance;
    Rotation2d wheelAngle = module.getModuleAngle().rotateBy(Rotation2d.fromRadians(robotHeading));
    Translation2d deltaPosition = new Translation2d(wheelAngle.cos() * deltaEncDistance,
        wheelAngle.sin() * deltaEncDistance);

    double xCorrectionFactor = 1.0;
    double yCorrectionFactor = 1.0;

    if (Math.signum(deltaPosition.x()) == 1.0) {
      xCorrectionFactor = (8.782 - .89) / 9.44;

    } else if (Math.signum(deltaPosition.x()) == -1.0) {
      xCorrectionFactor = (8.782 - .89) / 9.46;
    }

    if (Math.signum(deltaPosition.y()) == 1.0) {
      yCorrectionFactor = -(-8 + .89) / 8.71;

    } else if (Math.signum(deltaPosition.y()) == -1.0) {
      yCorrectionFactor = -(-8 + .89) / 8.62;
    }

    deltaPosition = new Translation2d(deltaPosition.x() * xCorrectionFactor, deltaPosition.y() * yCorrectionFactor);
    Translation2d updatedPosition = props.position.translateBy(deltaPosition);
    Pose2d wheelPose = new Pose2d(updatedPosition, Rotation2d.fromRadians(robotHeading));
    props.estimatedRobotPose = wheelPose.transformBy(Pose2d.fromTranslation(props.startingPosition.inverse()));

    props.position = updatedPosition;
    props.previousEncDistance = currentEncDistance;
  }

  /**
   * Resets the poses of the modules to the given robot pose.
   *
   * @param robotPose The new robot pose.
   */
  private void resetModulePoses(Pose2d robotPose) {
    for (int i = 0; i < mModules.length; i++) {
      WheelProperties props = wheels[i];
      Translation2d modulePosition = robotPose
          .transformBy(Pose2d.fromTranslation(props.startingPosition))
          .getTranslation();
      props.position = modulePosition;
    }
  }

  /**
   * Resets the poses of the modules to their starting positions.
   */
  private void resetModulePoses() {
    for (int i = 0; i < mModules.length; i++) {
      WheelProperties props = wheels[i];
      props.position = props.startingPosition;
    }
  }

  /**
   * Resets the robot's pose to the given pose.
   *
   * @param pose The new robot pose.
   */
  public void resetPose(Pose2d pose) {
    inputs.pose = pose;
    resetModulePoses(inputs.pose);
  }

  public class WheelProperties {
    private double previousEncDistance = 0;
    private Translation2d position;
    private Translation2d startingPosition;
    private Pose2d estimatedRobotPose = new Pose2d();
  }

  /**
   * Returns the current robot pose.
   *
   * @return The current robot pose.
   */
  public Pose2d getRobotPose() {
    return inputs.pose;
  }

  /**
   * Returns the measured velocity of the robot.
   *
   * @return The measured velocity.
   */
  public Translation2d getMeasuredVelocity() {
    return inputs.velocity;
  }

  /**
   * Returns the current timestamp.
   *
   * @return The current timestamp.
   */
  public double getTimestamp() {
    return mTimestamp;
  }

  public double wheel0_x() {
    return wheels[0].position.x();
  }

  public double wheel0_y() {
    return wheels[0].position.y();
  }

  public double wheel1_x() {
    return wheels[1].position.x();
  }

  public double wheel1_y() {
    return wheels[1].position.y();
  }

  public double wheel2_x() {
    return wheels[2].position.x();
  }

  public double wheel2_y() {
    return wheels[2].position.y();
  }

  public double wheel3_x() {
    return wheels[3].position.x();
  }

  public double wheel3_y() {
    return wheels[3].position.y();
  }

  public double robot_x() {
    return inputs.velocity.getTranslation().x();
  }

  public double robot_y() {
    return inputs.velocity.getTranslation().y();
  }
}
