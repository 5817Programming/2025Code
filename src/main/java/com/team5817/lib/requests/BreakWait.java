package com.team5817.lib.requests;

import com.team5817.lib.drivers.BeamBreak;
import com.team254.lib.util.TimeDelayedBoolean;
import com.team5817.lib.RobotMode;

public class BreakWait {
  /**
   * BeamBreak Sensor reading.
   * 
   * @param mBreak       BeamBreak Sensor.
   * @param target_state If wanted reading is true (broken) or false (not broken).
   * 
   * @return Boolean for if target state is acheived.
   */
  @SuppressWarnings("unused")
  private Request breakWait(BeamBreak mBreak, boolean target_state) {
    return new Request() {

      @Override
      public void act() {
      }

      @Override
      public boolean isFinished() {
        if (RobotMode.isSim()) {
          return true;
        }
        return mBreak.get() == target_state;
      }
    };
  }

  /**
   * Debounced BeamBreak Sensor reading.
   * 
   * @param mBreak               BeamBreak Sensor.
   * @param target_state         If wanted reading is true (broken) or false (not
   *                             broken).
   * @param delayed_wait_seconds Debounces time from a BeamBreak Sensor.
   * 
   * @return Boolean for if target state is acheived after debouncing the signal.
   */
  @SuppressWarnings("unused")
  private Request breakWait(BeamBreak mBreak, boolean target_state, double delayed_wait_seconds) {
    return new Request() {

      TimeDelayedBoolean timeout = new TimeDelayedBoolean();

      @Override
      public void act() {
      }

      @Override
      public boolean isFinished() {
        if (RobotMode.isSim()) {
          return timeout.update(true, delayed_wait_seconds);
        }

        return timeout.update(mBreak.get() == target_state, delayed_wait_seconds);
      }
    };
  }
}
