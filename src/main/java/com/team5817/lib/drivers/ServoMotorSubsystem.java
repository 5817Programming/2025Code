package com.team5817.lib.drivers;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team254.lib.drivers.CanDeviceId;
import com.team254.lib.drivers.Phoenix6Util;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.drivers.TalonUtil;
import com.team254.lib.motion.MotionState;
import com.team254.lib.util.DelayedBoolean;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.util.Util;
import com.team5817.frc2025.loops.ILooper;
import com.team5817.frc2025.loops.Loop;
import com.team5817.lib.RobotMode;
import com.team5817.lib.requests.Request;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.UnaryOperator;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

/**
 * Abstract base class for a subsystem with a single sensored servo-mechanism.
 * spotless:off
 */
public abstract class ServoMotorSubsystem extends Subsystem {
  protected static final int kMotionMagicSlot = 0;
  protected static final int kPositionPIDSlot = 1;

  // Recommend initializing in a static block!
  public static class TalonFXConstants {
    public CanDeviceId id = new CanDeviceId(-1);
    public boolean counterClockwisePositive = true;
    public boolean invert_sensor_phase = false;
  }

  // Recommend initializing in a static block!
  public static class ServoMotorSubsystemConstants {
    public String kName = "ERROR_ASSIGN_A_NAME";

    public double kLooperDt = 0.01;
    public double kCANTimeout = 0.010; // use for important on the fly updates
    public int kLongCANTimeoutMs = 100; // use for constructors

    public boolean simIO = false;

    public TalonFXConstants kMainConstants = new TalonFXConstants();
    public TalonFXConstants[] kFollowerConstants = new TalonFXConstants[0];

    public GravityTypeValue kGravityType = GravityTypeValue.Elevator_Static;
    public NeutralModeValue kNeutralMode = NeutralModeValue.Brake;
    public double kHomePosition = 0.0; // Units
    public double kRotationsPerUnitDistance = 1.0;
    public double kSoftLimitDeadband = 0.0;
    public double kKp = 0; // Raw output / raw error
    public double kKi = 0; // Raw output / sum of raw error
    public double kKd = 0; // Raw output / (err - prevErr)
    public double kKv = 0;
    public double kKa = 0;
    public double kKs = 0;
    public double kKg = 0;
    public int kDeadband = 0; // rotation

    public double kPositionKp = 0;
    public double kPositionKi = 0;
    public double kPositionKd = 0;
    public double kPositionKf = 0;
    public int kPositionDeadband = 0; // Ticks

    public double kVelocityFeedforward = 0;
    public double kArbitraryFeedforward = 0;
    public double kCruiseVelocity = 0; // Units/s
    public double kAcceleration = 0; // Units/s^2
    public double kJerk = 0; // Units/s^3
    public double kRampRate = 0.0; // s

    public int kSupplyCurrentLimit = 60; // amps
    public int kSupplyCurrentThreshold = 60;// TODO look at what this should be add to roller if good
    public double kSupplyCurrentTimeout = 0.0; // Seconds TODO look at what this should be add to roller if good
    public boolean kEnableSupplyCurrentLimit = false;

    public int kStatorCurrentLimit = 40; // amps
    public boolean kEnableStatorCurrentLimit = false;

    public double kMaxForwardOutput = 12.0; // Volts
    public double kMaxReverseOutput = -12.0; // Voltsa

    public boolean kFollowerOpposeMasterDirection = false;

    public double kMaxUnitsLimit = Double.POSITIVE_INFINITY;
    public double kMinUnitsLimit = Double.NEGATIVE_INFINITY;

    public int kStatusFrame8UpdateRate = 1000;

    public double kHomingTimeout = 0;
    public double kHomingVelocityWindow = 0;
    public double kHomingOutput = 0;

  }

  protected final ServoMotorSubsystemConstants mConstants;
  protected final TalonFX mMain;
  protected final TalonFX[] mFollowers;

  protected boolean mHoming = false;
  protected DelayedBoolean mHomingDelay;

  protected double demand = 0;

  protected TalonFXConfiguration mMainConfig;
  protected final TalonFXConfiguration[] mFollowerConfigs;

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

  protected MotionState mMotionStateSetpoint = null;

  protected double mForwardSoftLimitRotations;
  protected double mReverseSoftLimitRotations;

  /**
   * Constructor for ServoMotorSubsystem.
   *
   * @param constants The constants for the subsystem.
   */
  protected ServoMotorSubsystem(final ServoMotorSubsystemConstants constants) {
    mConstants = constants;
    mHomingDelay = new DelayedBoolean(Timer.getFPGATimestamp(), mConstants.kHomingTimeout);
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

    mForwardSoftLimitRotations = (((mConstants.kMaxUnitsLimit - mConstants.kHomePosition)
        * mConstants.kRotationsPerUnitDistance)
        - mConstants.kSoftLimitDeadband);
    mMainConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = mForwardSoftLimitRotations;
    mMainConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

    mReverseSoftLimitRotations = (((mConstants.kMinUnitsLimit - mConstants.kHomePosition)
        * mConstants.kRotationsPerUnitDistance)
        + mConstants.kSoftLimitDeadband);
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
    mMainConfig.MotionMagic.MotionMagicCruiseVelocity = unitsToRotations(mConstants.kCruiseVelocity);
    mMainConfig.MotionMagic.MotionMagicAcceleration = unitsToRotations(mConstants.kAcceleration);
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
  public static class ServoInputs implements Sendable {
    // INPUTS
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

    @Override
    public void initSendable(SendableBuilder builder) {
      builder.addDoubleProperty("PositionRots", () -> position_rots, null);
      builder.addDoubleProperty("PositionUnits", () -> position_units, null);
      builder.addDoubleProperty("VelocityRpS", () -> velocity_rps, null);
      builder.addDoubleProperty("VelocityUnitspS", () -> velocity_unitspS, null);
      builder.addDoubleProperty("OutputVoltage", () -> output_voltage, null);
      builder.addDoubleProperty("StatorCurrent", () -> main_stator_current, null);
      builder.addDoubleProperty("SupplyCurrent", () -> main_supply_current, null);
      builder.addDoubleProperty("ActiveTrajectoryPosition", () -> active_trajectory_position, null);
    }
  }

  protected enum ControlState {
    OPEN_LOOP,
    MOTION_MAGIC,
    POSITION_PID,
    VOLTAGE

  }

  private double lastTimestamp = 0;
  protected ServoInputsAutoLogged mServoInputs = new ServoInputsAutoLogged();
  protected ControlState mControlState = ControlState.OPEN_LOOP;
  protected ReflectingCSVWriter<ServoInputs> mCSVWriter = null;
  protected boolean mHasBeenZeroed = false;
  protected StatusSignal<Integer> mMainStickyFault;
  private double lastPosRots = 0;

  /**
   * Reads the periodic inputs from the Talon.
   */
  @Override
  public void readPeriodicInputs() {
    mServoInputs.timestamp = Timer.getFPGATimestamp();
    double dt = mServoInputs.timestamp - lastTimestamp;

    if (mMain.hasResetOccurred()) {
      DriverStation.reportError(mConstants.kName + ": Talon Reset! ", false);
      mServoInputs.reset_occured = true;
      return;
    } else {
      mServoInputs.reset_occured = false;
    }

    mMainStickyFault = mMain.getStickyFaultField();

    mServoInputs.error_rotations = mMainClosedLoopError.asSupplier().get();
    mServoInputs.error_rotations = mMainClosedLoopError.asSupplier().get();
    mServoInputs.main_stator_current = mMainStatorCurrentSignal.asSupplier().get().in(Amps);
    mServoInputs.main_supply_current = mMainSupplyCurrentSignal.asSupplier().get().in(Amps);
    mServoInputs.output_voltage = mMainOutputVoltageSignal.asSupplier().get().in(Volts);
    mServoInputs.output_percent = mMainOutputPercentageSignal.asSupplier().get();
    mServoInputs.velocity_rps = mMainVelocitySignal.asSupplier().get().in(RotationsPerSecond);
    mServoInputs.rotor_position = rotationsToUnits(mMain.getPosition().getValue().in(Rotations));
    if (RobotMode.isSim() || mConstants.simIO) {
      mServoInputs.error_rotations = (demand - mServoInputs.position_rots);
      switch (mControlState) {
        case OPEN_LOOP:
          mServoInputs.position_rots += demand / dt;
          break;
        case MOTION_MAGIC:
        case POSITION_PID:
          mServoInputs.position_rots += mServoInputs.error_rotations * dt / 0.05;// bad guess at motion for sim
          break;
        case VOLTAGE:
          mServoInputs.position_rots += demand / dt / 1000;
          mServoInputs.output_voltage = demand;
        default:
          break;
      }
      if (mServoInputs.position_rots > unitsToRotations(mConstants.kMaxUnitsLimit)) {
        mServoInputs.position_rots = unitsToRotations(mConstants.kMaxUnitsLimit);
      } else if (mServoInputs.position_rots < unitsToRotations(mConstants.kMinUnitsLimit)) {
        mServoInputs.position_rots = unitsToRotations(mConstants.kMinUnitsLimit);
      }
      mServoInputs.velocity_rps = (mServoInputs.position_rots - lastPosRots) / dt;

    } else {
      mServoInputs.position_rots = mMainPositionSignal.asSupplier().get().in(Rotations);
    }
    mServoInputs.position_units = rotationsToHomedUnits(mServoInputs.position_rots);
    mServoInputs.velocity_unitspS = rotationsToHomedUnits(mServoInputs.velocity_rps);
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

    if (mCSVWriter != null) {
      mCSVWriter.add(mServoInputs);
    }
    lastTimestamp = Timer.getTimestamp();
    lastPosRots = mServoInputs.position_rots;

    Logger.processInputs(mConstants.kName, mServoInputs);
  }

  /**
   * Writes the periodic outputs to the Talon.
   */
  @Override
  public void writePeriodicOutputs() {
    if (mHoming) {
      setOpenLoop(mConstants.kHomingOutput);
      if (mHomingDelay.update(
          Timer.getFPGATimestamp(),
          Math.abs(getVelocity()) < mConstants.kHomingVelocityWindow)) {
        Logger.recordOutput("test", true);
        forceZero();
        mHasBeenZeroed = true;
        mHomingDelay = new DelayedBoolean(Timer.getFPGATimestamp(), mConstants.kHomingTimeout);
        setSetpointMotionMagic(mConstants.kHomePosition);
        mHoming = false;
      }
    }
    if (mConstants.simIO)
      return;
    if (mControlState == ControlState.MOTION_MAGIC) {
      mMain.setControl(new MotionMagicVoltage(demand).withSlot(kMotionMagicSlot));
    } else if (mControlState == ControlState.POSITION_PID) {
      mMain.setControl(new PositionDutyCycle(demand).withSlot(kPositionPIDSlot));
    } else if (mControlState == ControlState.VOLTAGE) {
      mMain.setControl(new VoltageOut(demand));
    } else {
      mMain.setControl(new DutyCycleOut(demand));
    }
  }

  /**
   * Handles the main reset.
   *
   * @param reset Whether a reset occurred.
   */
  public void handleMainReset(boolean reset) {
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
        if (mServoInputs.reset_occured) {
          System.out.println(mConstants.kName + ": Main Talon reset occurred; resetting frame rates.");
          Phoenix6Util.checkErrorAndRetry(() -> mMainPositionSignal.setUpdateFrequency(200, 0.05));
          Phoenix6Util.checkErrorAndRetry(() -> mMainVelocitySignal.setUpdateFrequency(200, 0.05));
          Phoenix6Util.checkErrorAndRetry(() -> mMainClosedLoopError.setUpdateFrequency(200, 0.05));
          Phoenix6Util.checkErrorAndRetry(() -> mMainStatorCurrentSignal.setUpdateFrequency(200, 0.05));
          Phoenix6Util.checkErrorAndRetry(() -> mMainOutputVoltageSignal.setUpdateFrequency(200, 0.05));
          Phoenix6Util.checkErrorAndRetry(() -> mMainOutputPercentageSignal.setUpdateFrequency(200, 0.05));

          resetIfAtHome();
        }
        handleMainReset(mServoInputs.reset_occured);
        for (TalonFX follower : mFollowers) {
          if (follower.hasResetOccurred()) {
            System.out.println(mConstants.kName + ": Follower Talon reset occurred");
          }
        }
      }
    });
  }

  /**
   * Gets the position in rotations.
   *
   * @return The position in rotations.
   */
  public double getPositionRotations() {
    return mServoInputs.position_rots;
  }

  /**
   * Gets the position in units.
   *
   * @return The position in units.
   */
  public double getPosition() {
    return mServoInputs.position_units;
  }

  /**
   * Gets the velocity in units per second.
   *
   * @return The velocity in units per second.
   */
  public double getVelocity() {
    return rotationsToUnits(mServoInputs.velocity_rps);
  }

  /**
   * Gets the pure velocity in rotations per second.
   *
   * @return The pure velocity in rotations per second.
   */
  public double getPureVelocity() {
    return mServoInputs.velocity_rps;
  }

  /**
   * Gets the velocity error.
   *
   * @return The velocity error.
   */
  public double getVelError() {
    if (mMotionStateSetpoint == null) {
      return 0.0;
    }
    return rotationsToUnits(mMotionStateSetpoint.vel() - mServoInputs.velocity_rps);
  }

  /**
   * Checks if the trajectory has finished.
   *
   * @return True if the trajectory has finished, false otherwise.
   */
  public boolean hasFinishedTrajectory() {
    return Util.epsilonEquals(
        mServoInputs.active_trajectory_position, getSetpoint(), Math.max(1, mConstants.kDeadband));
  }

  /**
   * Gets the setpoint in units.
   *
   * @return The setpoint in units.
   */
  public double getSetpoint() {
    return (mControlState == ControlState.MOTION_MAGIC || mControlState == ControlState.POSITION_PID)
        ? rotationsToHomedUnits(demand)
        : Double.NaN;
  }

  /**
   * Gets the setpoint in homed units.
   *
   * @return The setpoint in homed units.
   */
  public double getSetpointHomed() {
    return (mControlState == ControlState.MOTION_MAGIC || mControlState == ControlState.POSITION_PID)
        ? rotationsToHomedUnits(demand)
        : Double.NaN;
  }

  /**
   * Sets the setpoint for motion magic.
   *
   * @param units         The setpoint in units.
   * @param feedforward_v The feedforward voltage.
   */
  public void setSetpointMotionMagic(double units, double feedforward_v) {
    demand = constrainRotations(homeAwareUnitsToRotations(units));
    if (mControlState != ControlState.MOTION_MAGIC) {
      mControlState = ControlState.MOTION_MAGIC;
    }
  }

  /**
   * Sets the setpoint for motion magic.
   *
   * @param units The setpoint in units.
   */
  public void setSetpointMotionMagic(double units) {
    setSetpointMotionMagic(units, 0.0);
  }

  /**
   * Sets the setpoint for position PID.
   *
   * @param units         The setpoint in units.
   * @param feedforward_v The feedforward voltage.
   */
  public void setSetpointPositionPID(double units, double feedforward_v) {
    demand = constrainRotations(homeAwareUnitsToRotations(units));
    if (mControlState != ControlState.POSITION_PID) {
      mControlState = ControlState.POSITION_PID;
    }
  }

  /**
   * Sets the setpoint for position PID.
   *
   * @param units The setpoint in units.
   */
  public void setSetpointPositionPID(double units) {
    setSetpointPositionPID(units, 0.0);
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
   * Converts rotations to homed units.
   *
   * @param rotations The rotations.
   * @return The homed units.
   */
  protected double rotationsToHomedUnits(double rotations) {
    double val = rotationsToUnits(rotations);
    return val + mConstants.kHomePosition;
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
   * Converts home-aware units to rotations.
   *
   * @param units The units.
   * @return The rotations.
   */
  protected double homeAwareUnitsToRotations(double units) {
    return unitsToRotations(units - mConstants.kHomePosition);
  }

  /**
   * Constrains the rotations within the soft limits.
   *
   * @param rotations The rotations.
   * @return The constrained rotations.
   */
  protected double constrainRotations(double rotations) {
    return Util.limit(rotations, mReverseSoftLimitRotations, mForwardSoftLimitRotations);
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
   * Gets the active trajectory position.
   *
   * @return The active trajectory position.
   */
  public double getActiveTrajectoryPosition() {
    return rotationsToHomedUnits((mServoInputs.active_trajectory_position));
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
   * Gets the predicted position in units after a lookahead time.
   *
   * @param lookahead_secs The lookahead time in seconds.
   * @return The predicted position in units.
   */
  public double getPredictedPositionUnits(double lookahead_secs) {
    double predicted_units = mServoInputs.active_trajectory_position
        + lookahead_secs * mServoInputs.active_trajectory_velocity
        + 0.5 * mServoInputs.active_trajectory_acceleration * lookahead_secs * lookahead_secs;
    if (demand >= mServoInputs.active_trajectory_position) {
      return Math.min(predicted_units, demand);
    } else {
      return Math.max(predicted_units, demand);
    }
  }

  /**
   * Returns a request to wait for the elevator to extend to the given position.
   * 
   * @param position the position to wait for
   * @return a request to wait for the elevator to extend
   */
  public Request waitToBeOverRequest(double position) {
    return new Request() {
      @Override
      public void act() {
      }

      @Override
      public boolean isFinished() {
        return mServoInputs.position_units >= position;
      }
    };
  }

  public void setWantHome(boolean home) {
    mHoming = home;
  }

  public void home() {
    setWantHome(true);
  }

  /**
   * Checks if the mechanism is at the homing location.
   *
   * @return True if at the homing location, false otherwise.
   */
  public boolean atHomingLocation() {
    return false;
  }

  /**
   * Resets the sensors if at the homing location.
   */
  public void resetIfAtHome() {
    if (atHomingLocation()) {
      zeroSensors();
    }
  }

  /**
   * Zeros the sensors.
   */
  @Override
  public void zeroSensors() {
    mMain.setPosition(0, mConstants.kCANTimeout);
    mHasBeenZeroed = true;
  }

  /**
   * Forces the sensors to zero.
   */
  public void forceZero() {
    Phoenix6Util.checkErrorAndRetry(() -> mMain.setPosition(0, mConstants.kCANTimeout));
  }

  /**
   * Checks if the sensors have been zeroed.
   *
   * @return True if the sensors have been zeroed, false otherwise.
   */
  public boolean hasBeenZeroed() {
    return mHasBeenZeroed;
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
   * Sets the motion magic configurations without checking the configuration.
   *
   * @param accel The acceleration.
   * @param jerk  The jerk.
   */
  public void setMotionMagicConfigsUnchecked(double accel, double jerk) {
    mMainConfig.MotionMagic.MotionMagicAcceleration = accel;
    mMainConfig.MotionMagic.MotionMagicJerk = jerk;
    mMain.getConfigurator().apply(mMainConfig.MotionMagic);
  }

  /**
   * Sets the motion magic configurations.
   *
   * @param accel    The acceleration.
   * @param velocity The cruise velocity.
   */
  public void setMotionMagicConfigs(double accel, double velocity) {
    mMainConfig.MotionMagic.MotionMagicAcceleration = unitsToRotations(accel);
    mMainConfig.MotionMagic.MotionMagicCruiseVelocity = unitsToRotations(velocity);

    TalonUtil.applyAndCheckConfiguration(mMain, mMainConfig);
  }

  /**
   * Outputs telemetry data.
   */
  @Override
  public void outputTelemetry() {
    Logger.recordOutput(mConstants.kName + "/Control Mode", mControlState);
    Logger.recordOutput(mConstants.kName + "/Demand", demand);
    Logger.recordOutput(mConstants.kName + "/Homing", mHoming);
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
