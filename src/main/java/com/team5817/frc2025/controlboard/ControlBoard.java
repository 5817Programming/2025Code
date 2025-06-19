package com.team5817.frc2025.controlboard;

import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team5817.frc2025.RobotConstants;
import com.team5817.frc2025.subsystems.Drive.Drive;
import com.team5817.lib.Util;

import edu.wpi.first.wpilibj.XboxController.Axis;

public class ControlBoard {
  private final double kSwerveDeadband = RobotConstants.stickDeadband;
  private Drive d;

  public final CustomXboxController driver;
  public final CustomXboxController operator;

  public ControlBoard(Drive d) {
    this.d = d;
    driver = new CustomXboxController(0);
    operator = new CustomXboxController(RobotConstants.kButtonGamepadPort);
  }

  /**
   * Updates the state of the driver and operator controllers.
   */
  public void update() {
    driver.update();
    operator.update();
  }

  /* DRIVER METHODS */
  double scalar = 1;

  /**
   * Sets the scalar value for swerve drive.
   * 
   * @param scalar the scalar value to set
   */
  public void setSwerveScalar(double scalar) {
    this.scalar = scalar;
  }

  /**
   * Gets the swerve translation based on the driver's controller input.
   * 
   * @return the swerve translation as a Translation2d object
   */
  public Translation2d getSwerveTranslation() {
    double forwardAxis = driver.getRawAxis(Axis.kLeftY.value);
    double strafeAxis = driver.getRawAxis(Axis.kLeftX.value);

    double expoForwardAxis = forwardAxis;
    double expoStrafeAxis = strafeAxis;

    expoForwardAxis = com.team5817.frc2025.subsystems.Drive.SwerveConstants.invertYAxis ? expoForwardAxis
        : -expoForwardAxis;
    expoStrafeAxis = com.team5817.frc2025.subsystems.Drive.SwerveConstants.invertXAxis ? expoStrafeAxis
        : -expoStrafeAxis;

    Translation2d tAxes = new Translation2d(expoForwardAxis, expoStrafeAxis).scale(scalar);

    if (Math.abs(tAxes.norm()) < kSwerveDeadband) {
      return new Translation2d();
    } else {
      Rotation2d deadband_direction = new Rotation2d(tAxes.x(), tAxes.y(), true);
      Translation2d deadband_vector = Translation2d.fromPolar(deadband_direction, kSwerveDeadband);

      double scaled_x = Util.scaledDeadband(expoForwardAxis, 1.0, Math.abs(deadband_vector.x()));
      double scaled_y = Util.scaledDeadband(expoStrafeAxis, 1.0, Math.abs(deadband_vector.y()));
      return new Translation2d(scaled_x, scaled_y)
          .scale(d.getMaxLinearSpeedMetersPerSec());
    }
  }

  /**
   * Gets the swerve rotation based on the driver's controller input.
   * 
   * @return the swerve rotation value
   */
  public double getSwerveRotation() {
    double rotAxis = driver.getRightX() * 0.80;
    rotAxis = com.team5817.frc2025.subsystems.Drive.SwerveConstants.invertRAxis ? rotAxis : -rotAxis;

    if (Math.abs(rotAxis) < kSwerveDeadband) {
      return 0.0;
    } else {
      return 8
          * (rotAxis - (Math.signum(rotAxis) * kSwerveDeadband))
          / (1 - kSwerveDeadband);
    }
  }

  /**
   * Checks if the gyro should be zeroed based on the driver's controller input.
   * 
   * @return true if the gyro should be zeroed, false otherwise
   */
  public boolean zeroGyro() {
    return driver.startButton.isBeingPressed() && driver.backButton.isBeingPressed();
  }

  // Only Driver

  /**
   * Checks if the top buttons on the driver's controller are clear.
   * 
   * @return true if the top buttons are clear, false otherwise
   */
  public boolean topButtonsClearDriver() {
    return !(driver.leftBumper.isBeingPressed()
        || driver.rightBumper.isBeingPressed()
        || driver.leftTrigger.isBeingPressed()
        || driver.rightTrigger.isBeingPressed());
  }

  // Driver and Operator

  /**
   * Checks if the top buttons on the operator's controller are clear.
   * 
   * @return true if the top buttons are clear, false otherwise
   */
  public boolean topButtonsClearOperator() {
    return !(operator.leftBumper.isBeingPressed()
        || operator.rightBumper.isBeingPressed()
        || operator.leftTrigger.isBeingPressed()
        || operator.rightTrigger.isBeingPressed());
  }

}
