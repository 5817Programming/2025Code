package com.team5817.lib.drivers;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team5817.lib.drivers.ServoMotorSubsystem.ControlState;

public interface ServoMotorIO {

  @AutoLog
  public static class ServoMotorIOInputs {
    public double timestamp;
    public double position_rots = 0; // motor rotations
    public double position_units;
    public double velocity_rps;
    public double velocity_unitspS;
    public double prev_vel_rps;
    public double output_percent;
    public double output_voltage;
    public double main_stator_current;
    public double main_supply_current;
    public double error_rotations;
    public boolean reset_occured;
    public double active_trajectory_position;
    public double active_trajectory_velocity;
    public double active_trajectory_acceleration;
    public double rotor_position;
  }

  default public ServoConstants getConstants() {return new ServoConstants();}

  default public void updateInputs(ServoMotorIOInputs inputs) {
  }

  default public void setControl(ServoMotorSubsystem.ControlState mControlState, double demand) {
    if (mControlState == ControlState.POSITION) {
      runPosition(demand);
    } else if (mControlState == ControlState.VOLTAGE) {
      runVoltage(demand);
    }

  }

  default public void runPosition(double units) {
  }

  default public void runVoltage(double volts) {
  }

  default public void zeroSensors() {
    zeroSensors(0);
  }

  default public void zeroSensors(double newPose) {
  }

  default public void forceZeroSensors() {
  }

  default public void setNeutralMode(NeutralModeValue mode) {

  }

  default public void setStatorCurrentLimit(double limit, boolean enable) {
  }

  default public void writeConfigs() {
  }

  default public boolean checkDeviceConfiguration() {
    return true;
  }
}
