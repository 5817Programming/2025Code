// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5817.frc2025.subsystems.Rollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.team254.lib.drivers.CanDeviceId;
import com.team254.lib.drivers.Phoenix6Util;
import com.team254.lib.drivers.TalonFXFactory;
import com.team5817.lib.util.PhoenixUtil;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/**
 * Generic roller IO implementation for a roller or series of rollers using a
 * Kraken.
 */
public class RollerSubsystemIOTalonFX implements RollerSubsystemIO {
  private final TalonFX mMain;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> tempCelsius;
  private final StatusSignal<Boolean> tempFault;

  // Single shot for voltage mode, robot loop will call continuously
  private final VoltageOut voltageOut = new VoltageOut(0.0).withUpdateFreqHz(0);
  private final MotionMagicVelocityTorqueCurrentFOC velocityOut = new MotionMagicVelocityTorqueCurrentFOC(0)
      .withUpdateFreqHz(0).withSlot(0);
  private final TorqueCurrentFOC torqueCurrentOut = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0);

  private final TalonFXConfiguration config;
  private final double reduction;

  public RollerSubsystemIOTalonFX(
      CanDeviceId id, RollerConstantsTalonFX mConstants, double reduction) {
    this.reduction = reduction;
    mMain = new TalonFX(id.getDeviceNumber(), id.getBus());

    Phoenix6Util.checkErrorAndRetry(() -> mMain.getBridgeOutput().setUpdateFrequency(200, 0.05));
    Phoenix6Util.checkErrorAndRetry(() -> mMain.getFault_Hardware().setUpdateFrequency(4, 0.05));

    config = TalonFXFactory.getDefaultConfig();

    config.Slot0.kP = mConstants.kKp;
    config.Slot0.kI = mConstants.kKi;
    config.Slot0.kD = mConstants.kKd;
    config.Slot0.kV = mConstants.kKv;
    config.Slot0.kA = mConstants.kKa;
    config.Slot0.kS = mConstants.kKs;

    config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = mConstants.kRampRate;
    config.OpenLoopRamps.VoltageOpenLoopRampPeriod = mConstants.kRampRate;
    config.OpenLoopRamps.TorqueOpenLoopRampPeriod = mConstants.kRampRate;

    config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = mConstants.kRampRate;
    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = mConstants.kRampRate;
    config.ClosedLoopRamps.TorqueClosedLoopRampPeriod = mConstants.kRampRate;
    config.CurrentLimits.SupplyCurrentLimit = mConstants.kSupplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = mConstants.kEnableSupplyCurrentLimit;

    config.CurrentLimits.StatorCurrentLimit = mConstants.kStatorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = mConstants.kEnableStatorCurrentLimit;

    config.Voltage.PeakForwardVoltage = mConstants.kMaxForwardOutput;
    config.Voltage.PeakReverseVoltage = mConstants.kMaxReverseOutput;

    config.MotorOutput.PeakForwardDutyCycle = mConstants.kMaxForwardOutput / 12.0;
    config.MotorOutput.PeakReverseDutyCycle = mConstants.kMaxReverseOutput / 12.0;

    config.MotorOutput.Inverted = (mConstants.counterClockwisePositive
        ? InvertedValue.CounterClockwise_Positive
        : InvertedValue.Clockwise_Positive);

    config.MotorOutput.NeutralMode = mConstants.kNeutralMode;
    PhoenixUtil.tryUntilOk(5, () -> mMain.getConfigurator().apply(config));

    position = mMain.getPosition();
    velocity = mMain.getVelocity();
    appliedVoltage = mMain.getMotorVoltage();
    supplyCurrent = mMain.getSupplyCurrent();
    torqueCurrent = mMain.getTorqueCurrent();
    tempCelsius = mMain.getDeviceTemp();
    tempFault = mMain.getFault_DeviceTemp();

    PhoenixUtil.tryUntilOk(
        5,
        () -> BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            position,
            velocity,
            appliedVoltage,
            supplyCurrent,
            torqueCurrent,
            tempCelsius,
            tempFault));
    PhoenixUtil.tryUntilOk(5, () -> mMain.optimizeBusUtilization(0, 1.0));

    // Register signals for refresh
    PhoenixUtil.registerSignals(
        new CANBus(id.getBus()).isNetworkFD(),
        position,
        velocity,
        appliedVoltage,
        supplyCurrent,
        torqueCurrent,
        tempCelsius,
        tempFault);
  }

  @Override
  public void updateInputs(RollerSubsystemIOInputs inputs) {
    inputs.data = new RollerSubsystemIOData(
        Units.rotationsToRadians(position.getValueAsDouble()) / reduction,
        Units.rotationsToRadians(velocity.getValueAsDouble()) / reduction,
        appliedVoltage.getValueAsDouble(),
        supplyCurrent.getValueAsDouble(),
        torqueCurrent.getValueAsDouble(),
        tempCelsius.getValueAsDouble(),
        tempFault.getValue(),
        BaseStatusSignal.isAllGood(
            position,
            velocity,
            appliedVoltage,
            supplyCurrent,
            torqueCurrent,
            tempCelsius,
            tempFault));
  }

  @Override
  public void runVolts(double volts) {
    mMain.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void runTorqueCurrent(double amps) {
    mMain.setControl(torqueCurrentOut.withOutput(amps));
  }

  @Override
  public void runVelocity(double velocity) {
    mMain.setControl(velocityOut.withVelocity(velocity));
  }

  @Override
  public void setCurrentLimit(double currentLimit) {
    new Thread(
        () -> {
          config.withCurrentLimits(config.CurrentLimits.withStatorCurrentLimit(currentLimit));
          PhoenixUtil.tryUntilOk(5, () -> mMain.getConfigurator().apply(config));
        });
  }
}
