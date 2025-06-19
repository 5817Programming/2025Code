package com.team5817.frc2025.autos.Actions;

import com.team5817.frc2025.subsystems.Superstructure;

import edu.wpi.first.wpilibj.Timer;

/**
 * Action to wait for a specified amount of time or until the superstructure
 * requests are completed.
 */
public class WaitForSuperstructureAction implements Action {
  private Superstructure mSuperstructure;
  private double mTimeToWait;
  private double mStartTime;

  /**
   * Constructor to wait for a specified amount of time.
   * 
   * @param timeToWait The time to wait in seconds.
   */
  public WaitForSuperstructureAction(double timeToWait, Superstructure s) {
    mSuperstructure = s;
    mTimeToWait = timeToWait;
  }

  /**
   * Default constructor to wait indefinitely until the superstructure requests
   * are completed.
   */
  public WaitForSuperstructureAction(Superstructure s) {
    mSuperstructure = s;
    mTimeToWait = Double.MAX_VALUE;
  }

  /**
   * Checks if the action is finished.
   * 
   * @return true if the specified time has passed or the superstructure requests
   *         are completed, false otherwise.
   */
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - mStartTime >= mTimeToWait || mSuperstructure.requestsCompleted();
  }

  /**
   * Starts the action by recording the start time.
   */
  @Override
  public void start() {
    mStartTime = Timer.getFPGATimestamp();
  }

  /**
   * Updates the action. No operation needed for this action.
   */
  @Override
  public void update() {
  }

  /**
   * Called once the action is done. No operation needed for this action.
   */
  @Override
  public void done() {
  }
}
