package com.team5817.lib.drivers;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team254.lib.drivers.CanDeviceId;
import com.team254.lib.drivers.Phoenix6Util;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.drivers.TalonUtil;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team5817.frc2025.loops.ILooper;
import com.team5817.frc2025.loops.Loop;
import com.team5817.lib.RobotMode;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.UnaryOperator;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

/**
 * Abstract base class for a subsystem with a single sensored servo-mechanism.
 * spotless:off
 */
public abstract class RollerSubsystemBasic extends Subsystem {
  protected static final int kVelocityPIDSlot = 0;

  // Recommend initializing in a static block!
  public static class TalonFXConstants {
    public CanDeviceId id = new CanDeviceId(-1);
    public boolean counterClockwisePositive = true;
  }

  // Recommend initializing in a static block!
  public static class RollerSubsystemConstants {
    public String kName = "ERROR_ASSIGN_A_NAME";

    public double kLooperDt = 0.01;
    public double kCANTimeout = 0.010; // use for important on the fly updates
    public int kLongCANTimeoutMs = 100; // use for constructors

    public boolean simIO = false;

    public TalonFXConstants kMainConstants = new TalonFXConstants();
    public TalonFXConstants[] kFollowerConstants = new TalonFXConstants[0];

    public NeutralModeValue kNeutralMode = NeutralModeValue.Brake;
    public double kRotationsPerUnitDistance = 1.0;
    public double kKp = 0; // Raw output / raw error
    public double kKi = 0; // Raw output / sum of raw error
    public double kKd = 0; // Raw output / (err - prevErr)
    public double kKv = 0;
    public double kKa = 0;
    public double kKs = 0;

    public double kRampRate = 0.0; // s

    public int kSupplyCurrentLimit = 60; // amps
    public boolean kEnableSupplyCurrentLimit = false;

    public int kStatorCurrentLimit = 40; // amps
    public boolean kEnableStatorCurrentLimit = false;

    public double kMaxForwardOutput = 12.0; // Volts
    public double kMaxReverseOutput = -12.0; // Voltsa

    public boolean kFollowerOpposeMasterDirection = false;

    public int kStatusFrame8UpdateRate = 1000;

    public double simSpeedMultiplyer = 1.0;
  }

  protected final RollerSubsystemConstants mConstants;
  protected final TalonFX mMain;
  protected final TalonFX[] mFollowers;

  protected boolean mHoming = false;

  protected double demand = 0;

  protected TalonFXConfiguration mMainConfig;
  protected final TalonFXConfiguration[] mFollowerConfigs;

  protected final StatusSignal<AngularVelocity> mMainVelocitySignal;
  protected final StatusSignal<Current> mMainStatorCurrentSignal;
  protected final StatusSignal<Current> mMainSupplyCurrentSignal;
  protected final StatusSignal<Voltage> mMainOutputVoltageSignal;
  protected final StatusSignal<Double> mMainOutputPercentageSignal;

  /**
   * Constructor for ServoMotorSubsystem.
   *
   * @param constants The constants for the subsystem.
   */
  protected RollerSubsystemBasic(final RollerSubsystemConstants constants) {
    mConstants = constants;
    mMain = TalonFXFactory.createDefaultTalon(mConstants.kMainConstants.id, false);
    mFollowers = new TalonFX[mConstants.kFollowerConstants.length];
    mFollowerConfigs = new TalonFXConfiguration[mConstants.kFollowerConstants.length];
    Phoenix6Util.checkErrorAndRetry(() -> mMain.getBridgeOutput().setUpdateFrequency(200, 0.05));
    Phoenix6Util.checkErrorAndRetry(() -> mMain.getFault_Hardware().setUpdateFrequency(4, 0.05));

    mMainVelocitySignal = mMain.getVelocity();
    mMainStatorCurrentSignal = mMain.getStatorCurrent();
    mMainSupplyCurrentSignal = mMain.getSupplyCurrent();
    mMainOutputVoltageSignal = mMain.getMotorVoltage();
    mMainOutputPercentageSignal = mMain.getDutyCycle();

    Phoenix6Util.checkErrorAndRetry(() -> mMainVelocitySignal.setUpdateFrequency(200, 0.05));
    Phoenix6Util.checkErrorAndRetry(() -> mMainStatorCurrentSignal.setUpdateFrequency(200, 0.05));
    Phoenix6Util.checkErrorAndRetry(() -> mMainOutputVoltageSignal.setUpdateFrequency(200, 0.05));
    Phoenix6Util.checkErrorAndRetry(() -> mMainOutputPercentageSignal.setUpdateFrequency(200, 0.05));

    mMainConfig = TalonFXFactory.getDefaultConfig();

    mMainConfig.Slot0.kP = mConstants.kKp;
    mMainConfig.Slot0.kI = mConstants.kKi;
    mMainConfig.Slot0.kD = mConstants.kKd;
    mMainConfig.Slot0.kV = mConstants.kKv;
    mMainConfig.Slot0.kA = mConstants.kKa;
    mMainConfig.Slot0.kS = mConstants.kKs;

    mMainConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = mConstants.kRampRate;
    mMainConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = mConstants.kRampRate;
    mMainConfig.OpenLoopRamps.TorqueOpenLoopRampPeriod = mConstants.kRampRate;

    mMainConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = mConstants.kRampRate;
    mMainConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = mConstants.kRampRate;
    mMainConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = mConstants.kRampRate;
    mMainConfig.CurrentLimits.SupplyCurrentLimit = mConstants.kSupplyCurrentLimit;
    mMainConfig.CurrentLimits.SupplyCurrentLimitEnable = mConstants.kEnableSupplyCurrentLimit;

    mMainConfig.CurrentLimits.StatorCurrentLimit = mConstants.kStatorCurrentLimit;
    mMainConfig.CurrentLimits.StatorCurrentLimitEnable = mConstants.kEnableStatorCurrentLimit;

    mMainConfig.Voltage.PeakForwardVoltage = mConstants.kMaxForwardOutput;
    mMainConfig.Voltage.PeakReverseVoltage = mConstants.kMaxReverseOutput;

    mMainConfig.MotorOutput.PeakForwardDutyCycle = mConstants.kMaxForwardOutput / 12.0;
    mMainConfig.MotorOutput.PeakReverseDutyCycle = mConstants.kMaxReverseOutput / 12.0;

    mMainConfig.MotorOutput.Inverted = (mConstants.kMainConstants.counterClockwisePositive
        ? InvertedValue.CounterClockwise_Positive
        : InvertedValue.Clockwise_Positive);

    mMainConfig.MotorOutput.NeutralMode = mConstants.kNeutralMode;

    for (int i = 0; i < mFollowers.length; ++i) {
      mFollowers[i] = TalonFXFactory.createPermanentFollowerTalon(
          mConstants.kFollowerConstants[i].id, mConstants.kMainConstants.id, mConstants.kFollowerOpposeMasterDirection);

      TalonFX follower = mFollowers[i];
      mFollowerConfigs[i] = new TalonFXConfiguration();
      TalonFXConfiguration followerConfig = mFollowerConfigs[i];
      Phoenix6Util.checkErrorAndRetry(() -> follower.getConfigurator().refresh(followerConfig));

      followerConfig.MotorOutput.Inverted = (mConstants.kMainConstants.counterClockwisePositive
          ? InvertedValue.CounterClockwise_Positive
          : InvertedValue.Clockwise_Positive);
      followerConfig.MotorOutput.NeutralMode = mConstants.kNeutralMode;
      followerConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
      followerConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
      follower.setControl(
          new Follower(mConstants.kMainConstants.id.getDeviceNumber(), mConstants.kFollowerOpposeMasterDirection));

      TalonUtil.applyAndCheckConfiguration(follower, followerConfig);
    }
    TalonUtil.applyAndCheckConfiguration(mMain, mMainConfig);

    // Send a neutral command.
    stop();
  }

  /**
   * Sets the stator current limit.
   *
   * @param currentLimit The current limit in amps.
   * @param enable       Whether to enable the current limit.
   */
  public void setStatorCurrentLimit(double currentLimit, boolean enable) {
    changeTalonConfig((conf) -> {
      conf.CurrentLimits.StatorCurrentLimit = currentLimit;
      conf.CurrentLimits.StatorCurrentLimitEnable = enable;
      return conf;
    });
  }

  /**
   * Enables or disables the soft limits.
   *
   * @param enable Whether to enable the soft limits.
   */
  public void enableSoftLimits(boolean enable) {
    UnaryOperator<TalonFXConfiguration> configChanger = (conf) -> {
      conf.SoftwareLimitSwitch.ForwardSoftLimitEnable = enable;
      conf.SoftwareLimitSwitch.ReverseSoftLimitEnable = enable;
      return conf;
    };
    mMainConfig = configChanger.apply(mMainConfig);

    configChanger = (conf) -> {
      conf.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
      conf.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
      return conf;
    };
    for (int i = 0; i < mFollowers.length; ++i) {
      mFollowerConfigs[i] = configChanger.apply(mFollowerConfigs[i]);
    }

    writeConfigs();
  }

  /**
   * Sets the neutral mode of the motor.
   *
   * @param mode The neutral mode.
   */
  public void setNeutralMode(NeutralModeValue mode) {
    changeTalonConfig((conf) -> {
      conf.MotorOutput.NeutralMode = mode;
      return conf;
    });
  }

  /**
   * Changes the Talon configuration.
   *
   * @param configChanger The function to change the configuration.
   */
  public void changeTalonConfig(UnaryOperator<TalonFXConfiguration> configChanger) {
    for (int i = 0; i < mFollowers.length; ++i) {
      mFollowerConfigs[i] = configChanger.apply(mFollowerConfigs[i]);
    }
    mMainConfig = configChanger.apply(mMainConfig);
    writeConfigs();
  }

  /**
   * Writes the configurations to the Talon.
   */
  public void writeConfigs() {
    for (int i = 0; i < mFollowers.length; ++i) {
      TalonFX follower = mFollowers[i];
      TalonFXConfiguration followerConfig = mFollowerConfigs[i];
      TalonUtil.applyAndCheckConfiguration(follower, followerConfig);
    }
    TalonUtil.applyAndCheckConfiguration(mMain, mMainConfig);
  }

  @AutoLog
  public static class RollerInputs implements Sendable {
    // INPUTS
    public double timestamp;
    public double velocity_rps;
    public double velocity_unitspS;
    public double output_percent;
    public double output_voltage;
    public double main_stator_current;
    public double main_supply_current;
    public boolean reset_occured;

    @Override
    public void initSendable(SendableBuilder builder) {
      builder.addDoubleProperty("VelocityRpS", () -> velocity_rps, null);
      builder.addDoubleProperty("VelocityUnitspS", () -> velocity_unitspS, null);
      builder.addDoubleProperty("OutputVoltage", () -> output_voltage, null);
      builder.addDoubleProperty("StatorCurrent", () -> main_stator_current, null);
      builder.addDoubleProperty("SupplyCurrent", () -> main_supply_current, null);
    }
  }

  public enum ControlState {
    OPEN_LOOP,
    VELOCITY,
    VOLTAGE
  }

  protected RollerInputsAutoLogged mRollerInputs = new RollerInputsAutoLogged();
  protected ControlState mControlState = ControlState.OPEN_LOOP;
  protected ReflectingCSVWriter<RollerInputs> mCSVWriter = null;
  protected boolean mHasBeenZeroed = false;
  protected StatusSignal<Integer> mMainStickyFault;

  /**
   * Reads the periodic inputs from the Talon.
   */
  @Override
  public void readPeriodicInputs() {
    mRollerInputs.timestamp = Timer.getFPGATimestamp();

    if (mMain.hasResetOccurred()) {
      DriverStation.reportError(mConstants.kName + ": Talon Reset! ", false);
      mRollerInputs.reset_occured = true;
      return;
    } else {
      mRollerInputs.reset_occured = false;
    }

    mMainStickyFault = mMain.getStickyFaultField();

    mRollerInputs.main_stator_current = mMainStatorCurrentSignal.asSupplier().get().in(Amps);
    mRollerInputs.main_supply_current = mMainSupplyCurrentSignal.asSupplier().get().in(Amps);
    mRollerInputs.output_voltage = mMainOutputVoltageSignal.asSupplier().get().in(Volts);
    mRollerInputs.output_percent = mMainOutputPercentageSignal.asSupplier().get();
    mRollerInputs.velocity_rps = mMainVelocitySignal.asSupplier().get().in(RotationsPerSecond);
    if (RobotMode.isSim() || mConstants.simIO) {
      switch (mControlState) {
        case OPEN_LOOP:
          mRollerInputs.velocity_rps = demand * mConstants.simSpeedMultiplyer;
          break;
        case VELOCITY:
          mRollerInputs.velocity_rps = demand;
          break;
        case VOLTAGE:
          mRollerInputs.velocity_rps = demand / 12 * mConstants.simSpeedMultiplyer;
          break;
      }

    }
    mRollerInputs.velocity_unitspS = rotationsToUnits(mRollerInputs.velocity_rps);

    if (mCSVWriter != null) {
      mCSVWriter.add(mRollerInputs);
    }
    Logger.processInputs(mConstants.kName, mRollerInputs);
  }

  /**
   * Writes the periodic outputs to the Talon.
   */
  @Override
  public void writePeriodicOutputs() {
    if (mConstants.simIO)
      return;

    switch (mControlState) {
      case OPEN_LOOP:
        mMain.setControl(new DutyCycleOut(demand));
        break;
      case VELOCITY:
        mMain.setControl(new VelocityDutyCycle(demand).withSlot(kVelocityPIDSlot));
        break;
      case VOLTAGE:
        mMain.setControl(new VoltageOut(demand));
        break;
    }
  }

  /**
   * Handles the main reset.
   *
   * @param reset Whether a reset occurred.
   */
  public void handleMainReset(boolean reset) {
    // TODO
  }

  /**
   * Registers the enabled loops.
   *
   * @param mEnabledLooper The enabled looper.
   */
  @Override
  public void registerEnabledLoops(ILooper mEnabledLooper) {
    mEnabledLooper.register(new Loop() {
      @Override
      public void onStart(double timestamp) {
      }

      @Override
      public void onLoop(double timestamp) {
        if (mRollerInputs.reset_occured) {
          System.out.println(mConstants.kName + ": Main Talon reset occurred; resetting frame rates.");
          Phoenix6Util.checkErrorAndRetry(() -> mMainVelocitySignal.setUpdateFrequency(200, 0.05));
          Phoenix6Util.checkErrorAndRetry(() -> mMainStatorCurrentSignal.setUpdateFrequency(200, 0.05));
          Phoenix6Util.checkErrorAndRetry(() -> mMainOutputVoltageSignal.setUpdateFrequency(200, 0.05));
          Phoenix6Util.checkErrorAndRetry(() -> mMainOutputPercentageSignal.setUpdateFrequency(200, 0.05));
        }
        handleMainReset(mRollerInputs.reset_occured);
        for (TalonFX follower : mFollowers) {
          if (follower.hasResetOccurred()) {
            System.out.println(mConstants.kName + ": Follower Talon reset occurred");
          }
        }
      }
    });
  }

  /**
   * Gets the velocity in units per second.
   *
   * @return The velocity in units per second.
   */
  public double getVelocity() {
    return rotationsToUnits(mRollerInputs.velocity_rps);
  }

  /**
   * Gets the pure velocity in rotations per second.
   *
   * @return The pure velocity in rotations per second.
   */
  public double getPureVelocity() {
    return mRollerInputs.velocity_rps;
  }

  /**
   * Converts rotations to units.
   *
   * @param rotations The rotations.
   * @return The units.
   */
  protected double rotationsToUnits(double rotations) {
    return rotations / mConstants.kRotationsPerUnitDistance;
  }

  /**
   * Converts units to rotations.
   *
   * @param units The units.
   * @return The rotations.
   */
  protected double unitsToRotations(double units) {
    return units * mConstants.kRotationsPerUnitDistance;
  }

  /**
   * Sets the open loop control.
   *
   * @param percentage The percentage output.
   */
  public void setOpenLoop(double percentage) {
    if (mControlState != ControlState.OPEN_LOOP) {
      mControlState = ControlState.OPEN_LOOP;
    }
    demand = percentage;
  }

  /**
   * Sets the open loop control.
   *
   * @param voltage The voltage output.
   */
  public void setVoltage(double voltage) {
    if (mControlState != ControlState.VOLTAGE) {
      mControlState = ControlState.VOLTAGE;
    }
    demand = voltage;
  }

  /**
   * Sets the open loop control.
   *
   * @param percentage The percentage output.
   */
  public void setVelocity(double units) {
    if (mControlState != ControlState.VELOCITY) {
      mControlState = ControlState.VELOCITY;
    }
    demand = units;
  }

  /**
   * Applies a voltage to the motor.
   *
   * @param voltage The voltage.
   */
  public void applyVoltage(double voltage) {
    if (mControlState != ControlState.VOLTAGE) {
      mControlState = ControlState.VOLTAGE;
    }
    demand = voltage;
  }

  /**
   * Gets the control state as a string.
   *
   * @return The control state.
   */
  public String getControlState() {
    return mControlState.toString();
  }

  /**
   * Sets the supply current limit.
   *
   * @param value  The current limit in amps.
   * @param enable Whether to enable the current limit.
   */
  public void setSupplyCurrentLimit(double value, boolean enable) {
    mMainConfig.CurrentLimits.SupplyCurrentLimit = value;
    mMainConfig.CurrentLimits.SupplyCurrentLimitEnable = enable;

    TalonUtil.applyAndCheckConfiguration(mMain, mMainConfig);
  }

  /**
   * Sets the supply current limit without checking the configuration.
   *
   * @param value  The current limit in amps.
   * @param enable Whether to enable the current limit.
   */
  public void setSupplyCurrentLimitUnchecked(double value, boolean enable) {
    mMainConfig.CurrentLimits.SupplyCurrentLimit = value;
    mMainConfig.CurrentLimits.SupplyCurrentLimitEnable = enable;

    mMain.getConfigurator().apply(mMainConfig);
  }

  /**
   * Sets the stator current limit without checking the configuration.
   *
   * @param value  The current limit in amps.
   * @param enable Whether to enable the current limit.
   */
  public void setStatorCurrentLimitUnchecked(double value, boolean enable) {
    mMainConfig.CurrentLimits.StatorCurrentLimit = value;
    mMainConfig.CurrentLimits.StatorCurrentLimitEnable = enable;

    mMain.getConfigurator().apply(mMainConfig);
  }

  /**
   * Outputs telemetry data.
   */
  @Override
  public void outputTelemetry() {
    Logger.recordOutput(mConstants.kName + "/Control Mode", mControlState);
  }

  /**
   * Rewrites the device configuration.
   */
  @Override
  public void rewriteDeviceConfiguration() {
    writeConfigs();
  }

  /**
   * Checks the device configuration.
   *
   * @return True if the configuration is correct, false otherwise.
   */
  @Override
  public boolean checkDeviceConfiguration() {
    if (!TalonUtil.readAndVerifyConfiguration(mMain, mMainConfig)) {
      return false;
    }
    for (int i = 0; i < mFollowers.length; ++i) {
      TalonFX follower = mFollowers[i];
      TalonFXConfiguration followerConfig = mFollowerConfigs[i];
      if (!TalonUtil.readAndVerifyConfiguration(follower, followerConfig)) {
        return false;
      }
    }
    return true;
  }
}
