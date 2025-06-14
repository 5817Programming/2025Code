package com.team5817.lib;

import lombok.Setter;

public class RobotMode {
  /**
   * Enum representing the different modes the robot can operate in.
   */
  public enum Mode {
    SIM,
    REPLAY,
    REAL
  }

  @Setter
  public static Mode mode = Mode.SIM;// Sim or Replay, Real is auto set for real robot

  public static boolean isReal() {
    return mode == Mode.REAL;
  }

  public static boolean isSim() {
    return mode == Mode.SIM;
  }

  public static boolean isReplay() {
    return mode == Mode.REPLAY;
  }

}
