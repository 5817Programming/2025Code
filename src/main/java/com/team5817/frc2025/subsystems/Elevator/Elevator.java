package com.team5817.frc2025.subsystems.Elevator;

import com.team5817.frc2025.RobotVisualizer;
import com.team5817.lib.Util;
import com.team5817.lib.drivers.ServoMotorIO;
import com.team5817.lib.drivers.ServoState;
import com.team5817.lib.drivers.StateBasedServoMotorSubsystem;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import lombok.Getter;

import org.littletonrobotics.junction.Logger;

/**
 * Elevator subsystem for controlling the elevator mechanism.
 */
public class Elevator extends StateBasedServoMotorSubsystem<Elevator.State> {

  /**
   * Constructs an Elevator with the given constants.
   * 
   * @param constants the constants for the elevator
   */
  public Elevator(ServoMotorIO io) {
    super(State.ZERO, io, false);
  }

  private double distanceFromScoringPosition = 0;
  private double scoringOffset = 0;

  final static double kStrictError = .01;
  final static double kMediumError = .1;
  final static double kLenientError = .2;

  /**
   * Enum representing the different states of the elevator.
   */
  public enum State implements ServoState {
    L4(1.8635330123363545, kStrictError, ElevatorConstants.kHighOffsetMap),
    L3(1.211806509200769 - .06, kStrictError, ElevatorConstants.kMidOffsetMap),
    L2(.754804 + .03, kStrictError, ElevatorConstants.kMidOffsetMap),
    L1(0.219, kStrictError),
    A1(0.59, kMediumError),
    A2(1, kMediumError),
    NET(2.035, kStrictError),
    ZERO(0, kLenientError),
    PROCESS(0.0, kLenientError),
    CLEAR(.35, kStrictError),
    STOW(0.0, kStrictError);

    @Getter
    private double demand = 0;
    @Getter
    private double allowableError = 20;
    private InterpolatingDoubleTreeMap map;

    State(double output, double allowable_error) {
      this(output, allowable_error, null);
    }

    State(double output, double allowable_error, InterpolatingDoubleTreeMap map) {
      this.demand = output;
      this.allowableError = allowable_error;
      this.map = map;
    }

    public double getTrackedOutput(double distanceFromScoringPosition) {
      if (map == null) {
        return demand;
      }
      double des = this.demand + map.get(distanceFromScoringPosition);
      des = Util.limit(des, ElevatorConstants.kElevatorServoConstants.kMinUnitsLimit,
          ElevatorConstants.kElevatorServoConstants.kMaxUnitsLimit);
      return des;
    }

    @Override
    public boolean isDisabled() {
      return false;
    }

    @Override
    public ControlState getControlState() {
      return ControlState.POSITION;
    }
  }

  public void updateOnBranchDistance(double dist) {
    this.distanceFromScoringPosition = dist;
  }

  public void setManualOffset(double offset) {
    this.scoringOffset = offset;
  }

  public void changeManualOffset(double deltaOffset) {
    this.scoringOffset += deltaOffset;
  }

  @Override
  public void writePeriodicOutputs() {
    double trackedOutput = mState.getTrackedOutput(distanceFromScoringPosition);
    if (mState == State.L1 || mState == State.L2 || mState == State.L3 || mState == State.L4)
      trackedOutput += scoringOffset;

    if (mControlState == ControlState.POSITION)
      setPositionSetpoint(trackedOutput);
    super.writePeriodicOutputs();
  }

  @Override
  public void outputTelemetry() {
    RobotVisualizer.updateElevatorHeight(getPosition());

    Logger.recordOutput("Elevator/Offset", this.scoringOffset);

    super.outputTelemetry();
  }
}
