package com.team5817.lib.drivers;

import org.littletonrobotics.junction.Logger;

import com.team5817.lib.Util;
import com.team5817.lib.requests.Request;

import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;

public class StateBasedServoMotorSubsystem<S extends Enum<S> & ServoState> extends ServoMotorSubsystem {
  @Getter
  @Setter
  @Accessors(prefix = "m")
  protected S mState;
  private final boolean allowAutoStateOutput;
  protected boolean atState = false;

  public StateBasedServoMotorSubsystem(S initialState, ServoMotorIO io,
      boolean enableAutoStateOutput) {
    super(io);
    this.mState = initialState;
    this.allowAutoStateOutput = enableAutoStateOutput;
  }

  public StateBasedServoMotorSubsystem(S initialState, ServoMotorIO io) {
    this(initialState, io, true);
  }

  @Override
  public void writePeriodicOutputs() {
    if (allowAutoStateOutput)
      switch (mState.getControlState()) {
        case POSITION:
          super.setPositionSetpoint(mState.getDemand());
          break;
        case VOLTAGE:
          super.applyVoltage(mState.getDemand());
      }

    if (mState.isDisabled())
      super.applyVoltage(0);

    super.writePeriodicOutputs();
  }

  @Override
  public void readPeriodicInputs() {
    super.readPeriodicInputs();
    atState = Util.epsilonEquals(getPosition(), mConstants.rotationsToUnits(demand), mState.getAllowableError());
    if (mState.isDisabled() || mControlState != ControlState.POSITION)
      atState = true;
  }

  @Override
  public void outputTelemetry() {
    Logger.recordOutput(mConstants.kName + "/AtState", atState);
    Logger.recordOutput(mConstants.kName + "/State", mState);
    super.outputTelemetry();
  }

  /**
   * Creates a request to change the state of the intake deployment.
   *
   * @param _wantedState The desired state.
   * @return The request to change the state.
   */
  public Request stateRequest(S _wantedState) {
    return new Request() {
      @Override
      public void act() {
        if (mControlState != ControlState.POSITION) {
          mControlState = ControlState.POSITION;
        }
        setState(_wantedState);
      }

      @Override
      public boolean isFinished() {
        return atState;
      }
    };
  }
}
