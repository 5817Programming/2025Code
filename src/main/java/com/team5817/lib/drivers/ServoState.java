package com.team5817.lib.drivers;

public interface ServoState {
  double getDemand();

  boolean isDisabled();

  default public double getAllowableError() {
    return Double.POSITIVE_INFINITY;
  }

  ServoMotorSubsystem.ControlState getControlState();
}
