package com.team5817.lib.drivers;

import com.team254.lib.drivers.CanDeviceId;
import com.team254.lib.motion.MotionState;
import com.team254.lib.util.DelayedBoolean;
import com.team254.lib.util.Util;
import com.team5817.lib.requests.Request;

import edu.wpi.first.wpilibj.Timer;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;

import org.littletonrobotics.junction.Logger;

/**
 * Abstract base class for a subsystem with a single sensored servo-mechanism.
 * spotless:off
 */
public abstract class ServoMotorSubsystem extends Subsystem {

  // Recommend initializing in a static block!
  public static class TalonFXConstants {
    public CanDeviceId id = new CanDeviceId(-1);
    public boolean counterClockwisePositive = true;
    public boolean invert_sensor_phase = false;
  }

  // Recommend initializing in a static block!
  ServoConstants mConstants;

  @Setter
  @Accessors(prefix = "m")
  protected boolean mHoming = false;
  protected DelayedBoolean mHomingDebounce;

  protected double demand = 0;

  protected MotionState mMotionStateSetpoint = null;

  ServoMotorIO io;

  /**
   * Constructor for ServoMotorSubsystem.
   *
   * @param constants The constants for the subsystem.
   */
  protected ServoMotorSubsystem(ServoMotorIO io) {
    this.io = io;
    mConstants = io.getConstants();
    mHomingDebounce = new DelayedBoolean(Timer.getFPGATimestamp(), mConstants.kHomingTimeout);
  }

  protected enum ControlState {
    POSITION,
    VOLTAGE
  }

  protected ServoMotorIOInputsAutoLogged mServoInputs = new ServoMotorIOInputsAutoLogged();
  @Getter
  @Accessors(prefix = "m")
  protected ControlState mControlState = ControlState.VOLTAGE;

  /**
   * Reads the periodic inputs from the Talon.
   */
  @Override
  public void readPeriodicInputs() {
    io.updateInputs(mServoInputs);
    Logger.processInputs(mConstants.kName, mServoInputs);
  }

  /**
   * Writes the periodic outputs to the Talon.
   */
  @Override
  public void writePeriodicOutputs() {
    if (mHoming)
      handleHoming();
    io.setControl(mControlState, demand);
  }

  public void handleHoming() {
    applyVoltage(mConstants.kHomingOutput * 12);
    if (mHomingDebounce.update(
        Timer.getFPGATimestamp(),
        Math.abs(getVelocity()) < mConstants.kHomingVelocityWindow)) {
      forceZero();
      mHomingDebounce = new DelayedBoolean(Timer.getFPGATimestamp(), mConstants.kHomingTimeout);
      setPositionSetpoint(mConstants.kHomePosition);
      mHoming = false;
    }
  }

  public void home() {
    setHoming(true);
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
    return mConstants.rotationsToUnits(mServoInputs.velocity_rps);
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
    return mConstants.rotationsToUnits(mMotionStateSetpoint.vel() - mServoInputs.velocity_rps);
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
    return mControlState == ControlState.POSITION
        ? mConstants.rotationsToHomedUnits(demand)
        : Double.NaN;
  }

  /**
   * Gets the setpoint in homed units.
   *
   * @return The setpoint in homed units.
   */
  public double getSetpointHomed() {
    return (mControlState == ControlState.POSITION)
        ? mConstants.rotationsToHomedUnits(demand)
        : Double.NaN;
  }

  /**
   * Sets the setpoint for motion magic.
   *
   * @param units The setpoint in units.
   */
  public void setPositionSetpoint(double units) {
    demand = constrainRotations(mConstants.homeAwareUnitsToRotations(units));
    if (mControlState != ControlState.POSITION) {
      mControlState = ControlState.POSITION;
    }
  }

  /**
   * Constrains the rotations within the soft limits.
   *
   * @param rotations The rotations.
   * @return The constrained rotations.
   */
  protected double constrainRotations(double rotations) {
    return Util.limit(rotations, mConstants.mReverseSoftLimitRotations, mConstants.mForwardSoftLimitRotations);
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
    return mConstants.rotationsToHomedUnits((mServoInputs.active_trajectory_position));
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

  /**
   * Zeros the sensors.
   */
  @Override
  public void zeroSensors() {
    io.zeroSensors();
  }

  /**
   * Forces the sensors to zero.
   */
  public void forceZero() {
    io.forceZeroSensors();
  }

  /**
   * Outputs telemetry data.
   */
  @Override
  public void outputTelemetry() {
    Logger.recordOutput(mConstants.kName + "/Control Mode", mControlState);
    Logger.recordOutput(mConstants.kName + "/Demand", mConstants.rotationsToUnits(demand));
    Logger.recordOutput(mConstants.kName + "/Homing", mHoming);
  }

  /**
   * Rewrites the device configuration.
   */
  @Override
  public void rewriteDeviceConfiguration() {
    io.writeConfigs();
  }

  /**
   * Checks the device configuration.
   *
   * @return True if the configuration is correct, false otherwise.
   */
  @Override
  public boolean checkDeviceConfiguration() {
    return io.checkDeviceConfiguration();
  }
}
