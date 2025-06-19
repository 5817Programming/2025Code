package com.team5817.lib.drivers;

import java.util.function.UnaryOperator;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team254.lib.drivers.Phoenix6Util;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.drivers.TalonUtil;
import com.team254.lib.util.Util;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class ServoMotorIOTalonFX implements ServoMotorIO {

  private final VoltageOut voltageControl = new VoltageOut(0);
  private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0).withSlot(0);

  protected final ServoConstants mConstants;
  protected final TalonFX mMain;
  protected final TalonFX[] mFollowers;

  protected StatusSignal<Integer> mMainStickyFault;

  protected TalonFXConfiguration mMainConfig;
  protected final TalonFXConfiguration[] mFollowerConfigs;

  protected double mForwardSoftLimitRotations;
  protected double mReverseSoftLimitRotations;

  protected final StatusSignal<Angle> mMainPositionSignal;
  protected final StatusSignal<AngularVelocity> mMainVelocitySignal;
  protected final StatusSignal<Double> mMainClosedLoopError;
  protected final StatusSignal<Current> mMainStatorCurrentSignal;
  protected final StatusSignal<Current> mMainSupplyCurrentSignal;
  protected final StatusSignal<Voltage> mMainOutputVoltageSignal;
  protected final StatusSignal<Double> mMainOutputPercentageSignal;
  protected final StatusSignal<Double> mMainClosedLoopOutputSignal;
  protected final StatusSignal<Double> mMainClosedLoopReferenceSignal;
  protected final StatusSignal<Double> mMainClosedLoopReferenceSlopeSignal;

  public ServoMotorIOTalonFX(ServoConstants mConstants) {
    this.mConstants = mConstants;
    mMain = TalonFXFactory.createDefaultTalon(mConstants.kMainConstants.id, false);
    mFollowers = new TalonFX[mConstants.kFollowerConstants.length];
    mFollowerConfigs = new TalonFXConfiguration[mConstants.kFollowerConstants.length];
    Phoenix6Util.checkErrorAndRetry(() -> mMain.getBridgeOutput().setUpdateFrequency(200, 0.05));
    Phoenix6Util.checkErrorAndRetry(() -> mMain.getFault_Hardware().setUpdateFrequency(4, 0.05));

    mMainPositionSignal = mMain.getPosition();
    mMainVelocitySignal = mMain.getVelocity();
    mMainClosedLoopError = mMain.getClosedLoopError();
    mMainStatorCurrentSignal = mMain.getStatorCurrent();
    mMainSupplyCurrentSignal = mMain.getSupplyCurrent();
    mMainOutputVoltageSignal = mMain.getMotorVoltage();
    mMainOutputPercentageSignal = mMain.getDutyCycle();
    mMainClosedLoopReferenceSignal = mMain.getClosedLoopReference();
    mMainClosedLoopOutputSignal = mMain.getClosedLoopOutput();
    mMainClosedLoopReferenceSlopeSignal = mMain.getClosedLoopReferenceSlope();

    Phoenix6Util.checkErrorAndRetry(() -> mMainPositionSignal.setUpdateFrequency(200, 0.05));
    Phoenix6Util.checkErrorAndRetry(() -> mMainVelocitySignal.setUpdateFrequency(200, 0.05));
    Phoenix6Util.checkErrorAndRetry(() -> mMainClosedLoopError.setUpdateFrequency(200, 0.05));
    Phoenix6Util.checkErrorAndRetry(() -> mMainStatorCurrentSignal.setUpdateFrequency(200, 0.05));
    Phoenix6Util.checkErrorAndRetry(() -> mMainOutputVoltageSignal.setUpdateFrequency(200, 0.05));
    Phoenix6Util.checkErrorAndRetry(() -> mMainOutputPercentageSignal.setUpdateFrequency(200, 0.05));
    Phoenix6Util.checkErrorAndRetry(() -> mMainClosedLoopReferenceSignal.setUpdateFrequency(200, 0.05));
    Phoenix6Util.checkErrorAndRetry(() -> mMainClosedLoopOutputSignal.setUpdateFrequency(200, 0.05));
    Phoenix6Util.checkErrorAndRetry(() -> mMainClosedLoopReferenceSlopeSignal.setUpdateFrequency(200, 0.05));

    mMainConfig = TalonFXFactory.getDefaultConfig();

    mMainConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    mMainConfig.Feedback.SensorToMechanismRatio = (mConstants.kMainConstants.invert_sensor_phase ? -1 : 1);

    mMainConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = mForwardSoftLimitRotations;
    mMainConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

    mMainConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = mReverseSoftLimitRotations;
    mMainConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    mMainConfig.Slot0.kP = mConstants.kKp;
    mMainConfig.Slot0.kI = mConstants.kKi;
    mMainConfig.Slot0.kD = mConstants.kKd;
    mMainConfig.Slot0.kV = mConstants.kKv;
    mMainConfig.Slot0.kG = mConstants.kKg;
    mMainConfig.Slot0.kA = mConstants.kKa;
    mMainConfig.Slot0.kS = mConstants.kKs;
    mMainConfig.Slot0.GravityType = mConstants.kGravityType;

    mMainConfig.Slot1.kP = mConstants.kPositionKp;
    mMainConfig.Slot1.kI = mConstants.kPositionKi;
    mMainConfig.Slot1.kD = mConstants.kPositionKd;
    mMainConfig.Slot1.kV = mConstants.kVelocityFeedforward;
    mMainConfig.MotionMagic.MotionMagicCruiseVelocity = mConstants.unitsToRotations(mConstants.kCruiseVelocity);
    mMainConfig.MotionMagic.MotionMagicAcceleration = mConstants.unitsToRotations(mConstants.kAcceleration);
    mMainConfig.MotionMagic.MotionMagicJerk = mConstants.kJerk;

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
  }

  @Override
  public void updateInputs(ServoMotorIOInputs mServoInputs) {
    mServoInputs.timestamp = Timer.getFPGATimestamp();

    if (mMain.hasResetOccurred()) {
      DriverStation.reportError(mConstants.kName + ": Talon Reset! ", false);
      mServoInputs.reset_occured = true;
      return;
    } else {
      mServoInputs.reset_occured = false;
    }

    mMainStickyFault = mMain.getStickyFaultField();

    mServoInputs.error_rotations = mMainClosedLoopError.asSupplier().get();
    mServoInputs.main_stator_current = mMainStatorCurrentSignal.asSupplier().get().in(Amps);
    mServoInputs.main_supply_current = mMainSupplyCurrentSignal.asSupplier().get().in(Amps);
    mServoInputs.output_voltage = mMainOutputVoltageSignal.asSupplier().get().in(Volts);
    mServoInputs.output_percent = mMainOutputPercentageSignal.asSupplier().get();
    mServoInputs.velocity_rps = mMainVelocitySignal.asSupplier().get().in(RotationsPerSecond);
    mServoInputs.rotor_position = mConstants.rotationsToUnits(mMain.getPosition().getValue().in(Rotations));
    mServoInputs.position_rots = mMainPositionSignal.asSupplier().get().in(Rotations);
    mServoInputs.position_units = mConstants.rotationsToHomedUnits(mServoInputs.position_rots);
    mServoInputs.velocity_unitspS = mConstants.rotationsToHomedUnits(mServoInputs.velocity_rps);
    mServoInputs.active_trajectory_position = mMainClosedLoopReferenceSignal.asSupplier().get();

    final double newVelocity = mMainClosedLoopReferenceSlopeSignal.asSupplier().get();
    if (Util.epsilonEquals(newVelocity, mConstants.kCruiseVelocity, Math.max(1, mConstants.kDeadband))
        || Util.epsilonEquals(
            newVelocity, mServoInputs.active_trajectory_velocity, Math.max(1, mConstants.kDeadband))) {
      // Mechanism is ~constant velocity.
      mServoInputs.active_trajectory_acceleration = 0.0;
    } else {
      // Mechanism is accelerating.
      mServoInputs.active_trajectory_acceleration = Math
          .signum(newVelocity - mServoInputs.active_trajectory_velocity) * mConstants.kAcceleration;
    }
    mServoInputs.active_trajectory_velocity = newVelocity;
  }

  /**
   * Sets the stator current limit.
   *
   * @param limit  The current limit in amps.
   * @param enable Whether to enable the current limit.
   */
  @Override
  public void setStatorCurrentLimit(double limit, boolean enable) {
    changeTalonConfig((conf) -> {
      conf.CurrentLimits.StatorCurrentLimit = limit;
      conf.CurrentLimits.StatorCurrentLimitEnable = enable;
      return conf;
    });
  }

  /**
   * Sets the neutral mode of the motor.
   *
   * @param mode The neutral mode.
   */
  @Override
  public void setNeutralMode(NeutralModeValue mode) {
    changeTalonConfig((conf) -> {
      conf.MotorOutput.NeutralMode = mode;
      return conf;
    });
  }

  @Override
  public void runVoltage(double volts) {
    mMain.setControl(voltageControl.withOutput(volts));
  }

  @Override
  public void runPosition(double demand) {
    mMain.setControl(positionControl.withPosition(demand));
  }

  /**
   * Zeros the sensors.
   */
  @Override
  public void zeroSensors(double newPos) {
    mMain.setPosition(newPos, mConstants.kCANTimeout);
  }

  /**
   * Forces the sensors to zero.
   */
  @Override
  public void forceZeroSensors() {
    Phoenix6Util.checkErrorAndRetry(() -> mMain.setPosition(0, mConstants.kCANTimeout));
  }

  @Override
  public ServoConstants getConstants() {
    return mConstants;
  }

  /**
   * Sets the motion magic configurations.
   *
   * @param accel    The acceleration.
   * @param velocity The cruise velocity.
   */
  public void setMotionMagicConfigs(double accel, double velocity) {
    mMainConfig.MotionMagic.MotionMagicAcceleration = mConstants.unitsToRotations(accel);
    mMainConfig.MotionMagic.MotionMagicCruiseVelocity = mConstants.unitsToRotations(velocity);
    TalonUtil.applyAndCheckConfiguration(mMain, mMainConfig);
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

  /**
   * Changes the Talon configuration.
   *
   * @param configChanger The function to change the configuration.
   */
  private void changeTalonConfig(UnaryOperator<TalonFXConfiguration> configChanger) {
    for (int i = 0; i < mFollowers.length; ++i) {
      mFollowerConfigs[i] = configChanger.apply(mFollowerConfigs[i]);
    }
    mMainConfig = configChanger.apply(mMainConfig);
    writeConfigs();
  }

  /**
   * Writes the configurations to the Talon.
   */
  @Override
  public void writeConfigs() {
    for (int i = 0; i < mFollowers.length; ++i) {
      TalonFX follower = mFollowers[i];
      TalonFXConfiguration followerConfig = mFollowerConfigs[i];
      TalonUtil.applyAndCheckConfiguration(follower, followerConfig);
    }
    TalonUtil.applyAndCheckConfiguration(mMain, mMainConfig);
  }
}
