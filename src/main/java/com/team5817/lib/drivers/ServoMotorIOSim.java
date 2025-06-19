package com.team5817.lib.drivers;

import com.team5817.lib.drivers.ServoMotorSubsystem.ControlState;

import edu.wpi.first.wpilibj.Timer;

public class ServoMotorIOSim implements ServoMotorIO {
  double tau = 0.05;
  ServoConstants mConstants;

  public ServoMotorIOSim(ServoConstants mConstants, double tau) {
    this.mConstants = mConstants;
    this.tau = tau;
  }

  public ServoMotorIOSim(ServoConstants mConstants) {
    this.mConstants = mConstants;
  }

  @Override
  public ServoConstants getConstants() {
    return mConstants;
  }

  double position_rots = 0;
  double demand = 0;
  ControlState mControlState = ControlState.POSITION;
  double lastTimestamp = 0;
  double lastPosRots = 0;

  @Override
  public void updateInputs(ServoMotorIOInputs inputs) {
    double dt = Timer.getTimestamp() - lastTimestamp;
    lastTimestamp = Timer.getTimestamp();

    inputs.error_rotations = (demand - inputs.position_rots);
    switch (mControlState) {
      case POSITION:
        inputs.position_rots += inputs.error_rotations * dt / tau;// bad guess at motion for sim
        break;
      case VOLTAGE:
        inputs.position_rots += demand / dt / 1000;
        inputs.output_voltage = demand;
    }
    if (inputs.position_rots > mConstants.unitsToRotations(mConstants.kMaxUnitsLimit)) {
      inputs.position_rots = mConstants.unitsToRotations(mConstants.kMaxUnitsLimit);
    } else if (inputs.position_rots < mConstants.unitsToRotations(mConstants.kMinUnitsLimit)) {
      inputs.position_rots = mConstants.unitsToRotations(mConstants.kMinUnitsLimit);
    }
    inputs.velocity_rps = (inputs.position_rots - lastPosRots) / dt;
    lastPosRots = inputs.position_rots;

    inputs.position_units = mConstants.rotationsToHomedUnits(inputs.position_rots);
    inputs.velocity_unitspS = mConstants.rotationsToHomedUnits(inputs.velocity_rps);

  }

  @Override
  public void runVoltage(double volts) {
    mControlState = ControlState.VOLTAGE;
    demand = volts;
  }

  @Override
  public void runPosition(double rotations) {
    mControlState = ControlState.POSITION;
    demand = rotations;
  }
}
