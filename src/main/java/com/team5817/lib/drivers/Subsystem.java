package com.team5817.lib.drivers;

import com.team5817.frc2025.loops.ILooper;

/**
 * The Subsystem abstract class, which serves as a basic framework for all robot
 * subsystems. Each subsystem outputs
 * commands to SmartDashboard, has a stop routine (for after each match), and a
 * routine to zero all sensors, which helps
 * with calibration.
 * <p>
 * All Subsystems only have one instance (after all, one robot does not have two
 * drivetrains), and functions get the
 * instance of the drivetrain and act accordingly. Subsystems are also a state
 * machine with a desired state and actual
 * state; the robot code will try to match the two states with actions. Each
 * Subsystem also is responsible for
 * instantializing all member components at the start of the match.
 */
public abstract class Subsystem {

  /**
   * Writes data to the log.
   */
  public void writeToLog() {
  }

  /**
   * Reads periodic inputs. This is an optional design pattern for caching
   * periodic reads to avoid hammering the HAL/CAN.
   */
  public void readPeriodicInputs() {
  }

  /**
   * Writes periodic outputs. This is an optional design pattern for caching
   * periodic writes to avoid hammering the HAL/CAN.
   */
  public void writePeriodicOutputs() {
  }

  /**
   * Stops the subsystem.
   */
  public void stop() {
  }

  /**
   * Zeros all sensors, which helps with calibration.
   */
  public void zeroSensors() {
  }

  /**
   * Outputs telemetry data.
   */
  public void outputTelemetry() {
  }

  /**
   * Registers enabled loops with the subsystem.
   * 
   * @param enabledLooper the looper to register
   */
  public void registerEnabledLoops(ILooper enabledLooper) {
  }

  /**
   * Checks the system for any issues.
   * 
   * @return true if the system is healthy, false otherwise
   */
  public boolean checkSystem() {
    return false;
  }

  public boolean hasEmergency = false;

  /**
   * Checks if attached devices have healthy configurations. This is an optional
   * pattern.
   * 
   * @return true if the device configurations are healthy, false otherwise
   */
  public boolean checkDeviceConfiguration() {
    return true;
  }

  /**
   * Rewrites the configuration of attached devices. This is an optional pattern.
   */
  public void rewriteDeviceConfiguration() {
  }
}
