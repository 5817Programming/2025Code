package com.team5817.frc2025.subsystems.Rollers;

import com.team5817.frc2025.subsystems.Rollers.RollerSubsystem.RollerControlMode;

public interface IRollerState {
  public double getDemand();

  public RollerControlMode getControlMode();
}
