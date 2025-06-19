package com.team5817.frc2025.autos.Actions;

import com.team5817.frc2025.field.FieldLayout;
import com.team5817.frc2025.subsystems.Drive.Drive;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * An action that waits until the robot passes a specific X coordinate.
 */
public class WaitToPassXCoordinateAction implements Action {
  double startingXCoordinate;
  double targetXCoordinate;
  Drive mDrive;

  /**
   * Constructs a new WaitToPassXCoordinateAction.
   *
   * @param x The target X coordinate to pass.
   */
  public WaitToPassXCoordinateAction(double x, Drive drive) {
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      targetXCoordinate = FieldLayout.kFieldLength - x;
    } else {
      targetXCoordinate = x;
    }
    mDrive = drive;
  }

  /**
   * Checks if the action is finished.
   *
   * @return true if the robot has passed the target X coordinate, false
   *         otherwise.
   */
  @Override
  public boolean isFinished() {
    return Math.signum(startingXCoordinate - targetXCoordinate) != Math
        .signum(mDrive.getPose().getTranslation().x() - targetXCoordinate);
  }

  /**
   * Starts the action by recording the starting X coordinate.
   */
  @Override
  public void start() {
    startingXCoordinate = mDrive.getPose().getTranslation().x();
  }

  /**
   * Updates the action. This method is called periodically while the action is
   * running.
   */
  @Override
  public void update() {
  }

  /**
   * Called once when the action is finished.
   */
  @Override
  public void done() {
  }
}
