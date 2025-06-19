// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5817.frc2025.subsystems.Rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollerSubsystemIO {
  @AutoLog
  class RollerSubsystemIOInputs {
    public RollerSubsystemIOData data = new RollerSubsystemIOData(0, 0, 0, 0, 0, 0, false, false);
  }

  record RollerSubsystemIOData(
      double positionRads,
      double velocityRadsPerSec,
      double appliedVoltage,
      double supplyCurrentAmps,
      double torqueCurrentAmps,
      double tempCelsius,
      boolean tempFault,
      boolean connected) {
  }

  default void updateInputs(RollerSubsystemIOInputs inputs) {
  }

  /* Run rollers at volts */
  default void runVolts(double volts) {
  }

  /* Run rollers at velocity */
  default void runVelocity(double volts) {
  }

  default void runTorqueCurrent(double amps) {
  }

  default void setCurrentLimit(double currentLimit) {
  }

  default void setBrakeMode(boolean enabled) {
  }
}
