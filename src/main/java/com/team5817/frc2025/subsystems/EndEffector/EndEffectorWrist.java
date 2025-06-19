package com.team5817.frc2025.subsystems.EndEffector;

import org.littletonrobotics.junction.Logger;

import com.team5817.frc2025.RobotVisualizer;
import com.team5817.frc2025.subsystems.EndEffector.EndEffectorConstants.EndEffectorWristConstants;
import com.team5817.lib.Util;
import com.team5817.lib.drivers.ServoMotorIO;
import com.team5817.lib.drivers.ServoState;
import com.team5817.lib.drivers.StateBasedServoMotorSubsystem;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import lombok.Getter;

public class EndEffectorWrist extends StateBasedServoMotorSubsystem<EndEffectorWrist.State> {

  private double distanceFromScoringLocation = 0;
  private double scoringOffset = 0;

  final static double kStrictError = 1.5;
  final static double kMediumError = 2;
  final static double kLenientError = 2.5;

  /**
   * Constructs an EndEffectorWrist with the given constants.
   * 
   * @param constants         The servo motor subsystem constants.
   * @param encoder_constants The absolute encoder constants.
   */
  public EndEffectorWrist(ServoMotorIO io) {
    super(State.ZERO, io, false);
  }

  public enum State implements ServoState {

    L4(71.89453125 - 89, kStrictError, EndEffectorWristConstants.kHighOffsetMap),
    L3(-31.185546874999993, kStrictError, EndEffectorWristConstants.kMidOffsetMap),
    L2(-40.185546874999993, kStrictError, EndEffectorWristConstants.kMidOffsetMap),
    L1(0 - 89, kStrictError),
    A1(180.342 - 89, kMediumError),
    A2(180.342 - 89, kMediumError),
    NET(175.342 - 89, kMediumError),
    ZERO(0 - 89, kMediumError),
    INTAKING(88.17, kStrictError),
    PINCH(131 - 89, 70),
    STOW(88.17, kStrictError);

    @Getter
    private double demand = 0;
    @Getter
    private double allowableError = 0;
    InterpolatingDoubleTreeMap map;

    State(double output, double allowable_error, InterpolatingDoubleTreeMap map) {
      this.demand = output;
      this.allowableError = allowable_error;
      this.map = map;
    }

    State(double output, double allowable_error) {
      this(output, allowable_error, null);
    }

    public double getTrackedOutput(double distanceFromScoringPosition) {
      if (map == null) {
        return demand;
      }
      double des = this.demand + map.get(distanceFromScoringPosition);
      des = Util.limit(des, EndEffectorWristConstants.kWristServoConstants.kMinUnitsLimit,
          EndEffectorWristConstants.kWristServoConstants.kMaxUnitsLimit);

      return des;
    }

    public boolean isDisabled() {
      return false;
    }

    @Override
    public ControlState getControlState() {
      return ControlState.POSITION;
    }
  }

  public void updateOnBranchDistance(double dist) {
    distanceFromScoringLocation = dist;
  }

  public void setManualOffset(double offset) {
    this.scoringOffset = offset;
  }

  public void changeManualOffset(Double deltaOffset) {
    this.scoringOffset += deltaOffset;
  }

  @Override
  public void writePeriodicOutputs() {
    double trackedOutput = mState.getTrackedOutput(distanceFromScoringLocation);
    if (mState == State.L1 || mState == State.L2 || mState == State.L3 || mState == State.L4)
      trackedOutput += scoringOffset;

    setPositionSetpoint(trackedOutput);

    super.writePeriodicOutputs();
  }

  @Override
  public void outputTelemetry() {
    RobotVisualizer.updateEndEffectorAngle(getPosition());

    Logger.recordOutput("EndEffectorWrist/Offset", this.scoringOffset);

    super.outputTelemetry();
  }
}
