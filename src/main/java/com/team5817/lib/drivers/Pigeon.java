package com.team5817.lib.drivers;

import static edu.wpi.first.units.Units.Degree;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import com.team254.lib.geometry.Rotation2d;
import com.team5817.frc2025.Ports;
import com.team5817.lib.RobotMode;

/**
 * The Pigeon class is responsible for interfacing with the Pigeon2 gyro sensor.
 * It provides methods to get and set the yaw, roll, and pitch angles.
 */
public class Pigeon {

  private static Pigeon mInstance;

  /**
   * Returns the singleton instance of the Pigeon class.
   *
   * @return The singleton instance of the Pigeon class.
   */
  public static Pigeon getInstance() {
    if (mInstance == null) {
      mInstance = new Pigeon(Ports.PIGEON.getDeviceNumber(), Ports.PIGEON.getBus());
    }
    return mInstance;
  }

  // Actual pigeon object
  private final Pigeon2 mGyro;

  // Configs
  private boolean inverted = com.team5817.frc2025.subsystems.Drive.SwerveConstants.invertGyro;
  private Rotation2d yawAdjustmentAngle = new Rotation2d();
  private Rotation2d rollAdjustmentAngle = new Rotation2d();
  private Rotation2d pitchAdjustmentAngle = new Rotation2d();

  private Pigeon(int port, String bus) {
    mGyro = new Pigeon2(port, bus);
    mGyro.getConfigurator().apply(new Pigeon2Configuration());
  }

  static SwerveDriveSimulation driveSim;

  /**
   * Registers the SwerveDriveSimulation instance for gyro simulation.
   *
   * @param sim The SwerveDriveSimulation instance.
   */
  public static void registerGyroSim(SwerveDriveSimulation sim) {
    driveSim = sim;
  }

  /**
   * Gets the current yaw angle, adjusted for any set offsets.
   *
   * @return The current yaw angle as a Rotation2d object.
   */
  public Rotation2d getYaw() {
    if (RobotMode.isSim()) {
      return Rotation2d.fromDegrees(driveSim.getSimulatedDriveTrainPose().getRotation().getDegrees());
    }
    Rotation2d angle = getUnadjustedYaw().rotateBy(yawAdjustmentAngle.inverse());
    if (inverted) {
      return angle.inverse();
    }
    return angle;
  }

  /**
   * Gets the current roll angle, adjusted for any set offsets.
   *
   * @return The current roll angle as a Rotation2d object.
   */
  public Rotation2d getRoll() {
    return getUnadjustedRoll().rotateBy(rollAdjustmentAngle.inverse());
  }

  /**
   * Gets the current pitch angle, adjusted for any set offsets.
   *
   * @return The current pitch angle as a Rotation2d object.
   */
  public Rotation2d getPitch() {
    if (RobotMode.isSim()) {
      return Rotation2d.identity();
    }
    return getUnadjustedPitch().rotateBy(pitchAdjustmentAngle.inverse()).inverse();
  }

  /**
   * Sets the yaw register to read the specified value.
   *
   * @param angleDeg New yaw in degrees.
   */
  public void setYaw(double angleDeg) {
    yawAdjustmentAngle = Rotation2d.fromDegrees(getYawStatusSignal().getValueAsDouble())
        .rotateBy(Rotation2d.fromDegrees(angleDeg).inverse());
  }

  /**
   * Sets the roll register to read the specified value.
   *
   * @param angleDeg New roll in degrees.
   */
  public void setRoll(double angleDeg) {
    rollAdjustmentAngle = getUnadjustedRoll().rotateBy(Rotation2d.fromDegrees(angleDeg).inverse());
  }

  /**
   * Sets the pitch register to read the specified value.
   *
   * @param angleDeg New pitch in degrees.
   */
  public void setPitch(double angleDeg) {
    pitchAdjustmentAngle = getUnadjustedPitch().rotateBy(Rotation2d.fromDegrees(angleDeg).inverse());
    System.out.println("Reset gyro to " + getPitch().getDegrees());
  }

  /**
   * Gets the unadjusted yaw angle directly from the sensor.
   *
   * @return The unadjusted yaw angle as a Rotation2d object.
   */
  public Rotation2d getUnadjustedYaw() {

    return Rotation2d.fromDegrees(
        BaseStatusSignal.getLatencyCompensatedValue(getYawStatusSignal(), getRateStatusSignal()).in(Degree));
  }

  /**
   * Gets the unadjusted pitch angle directly from the sensor.
   *
   * @return The unadjusted pitch angle as a Rotation2d object.
   */
  public Rotation2d getUnadjustedPitch() {
    return Rotation2d.fromDegrees(mGyro.getRoll().getValue().in(Degree));
  }

  /**
   * Gets the unadjusted roll angle directly from the sensor.
   *
   * @return The unadjusted roll angle as a Rotation2d object.
   */
  public Rotation2d getUnadjustedRoll() {
    return Rotation2d.fromDegrees(mGyro.getPitch().getValue().in(Degree));
  }

  /**
   * Gets the yaw status signal from the sensor.
   *
   * @return The yaw status signal as a StatusSignal<Angle> object.
   */
  public StatusSignal<Angle> getYawStatusSignal() {
    return mGyro.getYaw();
  }

  /**
   * Gets the angular velocity status signal from the sensor.
   *
   * @return The angular velocity status signal as a StatusSignal<AngularVelocity>
   *         object.
   */
  public StatusSignal<AngularVelocity> getRateStatusSignal() {
    return mGyro.getAngularVelocityZDevice();
  }
}
