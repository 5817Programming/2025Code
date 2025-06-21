package com.team5817.lib.drivers.Rollers;

import com.team5817.lib.drivers.Rollers.RollerSubsystem.RollerControlMode;

public interface IRollerState {
  public double getDemand();

  public RollerControlMode getControlMode();
}
