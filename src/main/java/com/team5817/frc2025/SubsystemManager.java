package com.team5817.frc2025;

import com.team5817.lib.drivers.Subsystem;

import java.util.Arrays;
import java.util.List;

/**
 * Used to reset, start, stop, and update all subsystems at once.
 */
public class SubsystemManager {
  public static SubsystemManager mInstance = null;

  private List<Subsystem> mAllSubsystems;

  private SubsystemManager() {
  }

  /**
   * Returns the singleton instance of the SubsystemManager.
   *
   * @return the singleton instance of the SubsystemManager.
   */
  public static SubsystemManager getInstance() {
    if (mInstance == null) {
      mInstance = new SubsystemManager();
    }

    return mInstance;
  }

  /**
   * Outputs telemetry data for all subsystems.
   */
  public void outputTelemetry() {
    if (RobotConstants.disableExtraTelemetry) {
      return;
    }
    mAllSubsystems.forEach(Subsystem::outputTelemetry);
  }

  /**
   * Checks the status of all subsystems.
   *
   * @return true if all subsystems are functioning correctly, false otherwise.
   */
  public boolean checkSubsystems() {
    boolean ret_val = true;

    for (Subsystem s : mAllSubsystems) {
      ret_val &= s.checkSystem();
    }

    return ret_val;
  }

  /**
   * Stops all subsystems.
   */
  public void stop() {
    mAllSubsystems.forEach(Subsystem::stop);
  }

  /**
   * Returns the list of all subsystems.
   *
   * @return the list of all subsystems.
   */
  public List<Subsystem> getSubsystems() {
    return mAllSubsystems;
  }

  /**
   * Sets the list of all subsystems.
   *
   * @param allSubsystems the subsystems to be managed.
   */
  public void setSubsystems(Subsystem... allSubsystems) {
    mAllSubsystems = Arrays.asList(allSubsystems);
  }

  public void updateSubsystems(){
    for(Subsystem s:mAllSubsystems)
      s.readPeriodicInputs();
    for(Subsystem s:mAllSubsystems)
      s.periodic();
    for(Subsystem s:mAllSubsystems)
      s.writePeriodicOutputs();
    outputTelemetry();
  }

public void start() {
    for(Subsystem s: mAllSubsystems)
      s.start();
}

}
