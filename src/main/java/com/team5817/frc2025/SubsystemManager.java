package com.team5817.frc2025;

import com.team5817.frc2025.loops.ILooper;
import com.team5817.frc2025.loops.Loop;
import com.team5817.frc2025.loops.Looper;
import com.team5817.lib.drivers.Subsystem;

import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Used to reset, start, stop, and update all subsystems at once.
 */
public class SubsystemManager implements ILooper {
  public static SubsystemManager mInstance = null;

  private List<Subsystem> mAllSubsystems;
  private List<Loop> mLoops = new ArrayList<>();
  private double read_dt = 0.0;
  private double on_loop_dt = 0.0;
  @SuppressWarnings("unused")
  private double write_dt = 0.0;

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

  private class EnabledLoop implements Loop {
    @Override
    public void onStart(double timestamp) {
      mLoops.forEach(l -> l.onStart(timestamp));
    }

    @Override
    public void onLoop(double timestamp) {
      // Read
      for (int i = 0; i < mAllSubsystems.size(); i++) {
        mAllSubsystems.get(i).readPeriodicInputs();
      }

      // On loop
      for (int i = 0; i < mLoops.size(); i++) {
        mLoops.get(i).onLoop(timestamp);
      }
      on_loop_dt = Timer.getTimestamp() - (timestamp + read_dt);

      // Write
      for (int i = 0; i < mAllSubsystems.size(); i++) {
        mAllSubsystems.get(i).writePeriodicOutputs();
      }
      write_dt = Timer.getTimestamp() - (timestamp + on_loop_dt);
      SubsystemManager.getInstance().outputTelemetry();

      // Telemetry
      outputTelemetry();
    }
  }

  /**
   * Registers the enabled loops for all subsystems.
   *
   * @param enabledLooper the looper to register the enabled loops with.
   */
  public void registerEnabledLoops(Looper enabledLooper) {
    mAllSubsystems.forEach(s -> s.registerEnabledLoops(this));
    enabledLooper.register(new EnabledLoop());
  }

  @Override
  public void register(Loop loop) {
    mLoops.add(loop);
  }

}
