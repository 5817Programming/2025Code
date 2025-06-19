package com.team5817.frc2025.autos.Actions;

import com.team5817.frc2025.RobotConstants;
import com.team5817.frc2025.field.FieldLayout;
import com.team5817.frc2025.subsystems.Drive.Drive;

/**
 * Action that waits until the robot has passed a certain distance to the reef.
 */
public class WaitToPassDistanceToReef implements Action {
  double startingDistance;
  double targetDistance;
  Drive mDrive;

  /**
   * Constructs a WaitToPassDistanceToReef action with the default distance.
   */
  public WaitToPassDistanceToReef(Drive drive) {
    this(RobotConstants.kDefaultDistanceToReef, drive);
  }

  /**
   * Constructs a WaitToPassDistanceToReef action with a specified distance.
   *
   * @param distance The target distance to the reef.
   */
  public WaitToPassDistanceToReef(double distance, Drive drive) {
    this.targetDistance = distance;
    this.mDrive = drive;
  }

  /**
   * Checks if the action is finished.
   *
   * @return true if the robot has passed the target distance to the reef, false
   *         otherwise.
   */
  @Override
  public boolean isFinished() {

    double currentDistance = mDrive.getPose().getTranslation()
        .translateBy(FieldLayout.getReefPose().inverse().getTranslation()).norm();
    return Math.signum(startingDistance - targetDistance) != Math.signum(currentDistance - targetDistance);
  }

  /**
   * Starts the action by recording the starting distance.
   */
  @Override
  public void start() {
    startingDistance = mDrive.getPose().getTranslation()
        .translateBy(FieldLayout.getReefPose().inverse().getTranslation()).norm();
  }

  /**
   * Updates the action. This method is empty as no updates are needed.
   */
  @Override
  public void update() {
  }

  /**
   * Called once the action is done. This method is empty as no cleanup is needed.
   */
  @Override
  public void done() {
  }
}
