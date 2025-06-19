package com.team5817.frc2025.subsystems.Intake;

import com.team5817.frc2025.RobotVisualizer;
import com.team5817.lib.drivers.ServoConstants;
import com.team5817.lib.drivers.ServoMotorIO;
import com.team5817.lib.drivers.ServoState;
import com.team5817.lib.drivers.StateBasedServoMotorSubsystem;

import lombok.Getter;

/**
 * The IntakeDeploy class controls the deployment mechanism of the intake
 * system.
 */
public class IntakeDeploy extends StateBasedServoMotorSubsystem<IntakeDeploy.State> {

  /**
   * Constructs a new IntakeDeploy subsystem.
   *
   * @param constants         The constants for the servo motor subsystem.
   * @param encoder_constants The constants for the absolute encoder.
   */
  public IntakeDeploy(final ServoConstants constants, ServoMotorIO io) {
    super(IntakeDeploy.State.GROUND, io);
  }

  final static double kStrictError = 20;
  final static double kMediumError = 50;
  final static double kLenientError = 80;

  /**
   * Represents the different states of the intake deployment.
   */
  public enum State implements ServoState {
    GROUND(-141, kStrictError),
    STOW(0, kMediumError),
    HUMAN(-141, kStrictError),
    ZERO(0, kStrictError),
    DISABLE();

    @Getter
    private double demand = 0;
    @Getter
    private double allowableError = 0;
    @Getter
    private boolean disabled = false;

    /**
     * p
     * Constructs a new State.
     *
     * @param output          The output value for the state.
     * @param allowable_error The allowable error for the state.
     */
    State(double output, double allowable_error) {
      this.demand = output;
      this.allowableError = allowable_error;
    }

    State() {
      this.disabled = true;
    }

    @Override
    public ControlState getControlState() {
      return ControlState.POSITION;
    }
  }

  /**
   * Outputs telemetry data for the subsystem.
   */
  @Override
  public void outputTelemetry() {
    RobotVisualizer.updateIntakeAngle(getPosition());

    super.outputTelemetry();
  }
}
