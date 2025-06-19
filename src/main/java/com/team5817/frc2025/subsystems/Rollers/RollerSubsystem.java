// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5817.frc2025.subsystems.Rollers;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;

import org.littletonrobotics.junction.Logger;

import com.team5817.lib.drivers.Subsystem;
import com.team5817.lib.requests.Request;

public class RollerSubsystem<S extends Enum<S> & IRollerState> extends Subsystem {
  private final String inputsName;
  private final RollerSubsystemIO io;
  protected final RollerSubsystemIOInputsAutoLogged inputs = new RollerSubsystemIOInputsAutoLogged();
  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert disconnected;
  private final Alert tempFault;

  private boolean brakeModeEnabled = true;

  @Getter
  @Setter
  @Accessors(prefix = "m")
  private S mState;

  public enum RollerControlMode {
    VOLTAGE,
    TORQUE,
    VELOCITY
  }

  public RollerSubsystem(S initialState, String name, String inputsName, RollerSubsystemIO io) {
    this.mState = initialState;
    this.inputsName = name;
    this.io = io;

    disconnected = new Alert(name + " motor disconnected!", Alert.AlertType.kWarning);
    tempFault = new Alert(name + " motor too hot! 🥵", Alert.AlertType.kWarning);
  }

  public void readPeriodicInputs() {
    io.updateInputs(inputs);
    Logger.processInputs(inputsName, inputs);
    disconnected.set(
        !motorConnectedDebouncer.calculate(inputs.data.connected()));
    tempFault.set(inputs.data.tempFault());
  }

  public void writePeriodicOutputs() {
    switch (mState.getControlMode()) {
      case TORQUE:
        io.runTorqueCurrent(mState.getDemand());
        break;
      case VELOCITY:
        io.runVelocity(mState.getDemand());
        break;
      case VOLTAGE:
        io.runVolts(mState.getDemand());
        break;
    }

    Logger.recordOutput(inputsName + "/BrakeModeEnabled", brakeModeEnabled);
  }

  public RollerSubsystem(S initialState, String name, RollerSubsystemIO io) {
    this(initialState, name, String.join(name, "Inputs"), io);
  }

  public void setBrakeMode(boolean enabled) {
    if (brakeModeEnabled == enabled)
      return;
    brakeModeEnabled = enabled;
    io.setBrakeMode(enabled);
  }

  public double getTorqueCurrent() {
    return inputs.data.torqueCurrentAmps();
  }

  public double getVelocity() {
    return inputs.data.velocityRadsPerSec();
  }

  public Request stateRequest(S newState) {
    return new Request() {
      @Override
      public void act() {
        setState(newState);
      }
    };
  }

  public boolean allOK() {
    return !disconnected.get() && !tempFault.get();
  }
}
