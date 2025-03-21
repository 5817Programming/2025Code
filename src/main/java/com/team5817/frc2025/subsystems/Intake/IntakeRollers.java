package com.team5817.frc2025.subsystems.Intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team254.lib.drivers.TalonUtil;
import com.team254.lib.util.TimeDelayedBoolean;
import com.team5817.frc2025.Ports;
import com.team5817.frc2025.Constants.IntakeRollerConstants;
import com.team5817.frc2025.loops.ILooper;
import com.team5817.frc2025.loops.Loop;
import com.team5817.lib.drivers.Subsystem;
import com.team5817.lib.requests.Request;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeRollers extends Subsystem {
	private static IntakeRollers mInstance;

	/**
	 * Gets the singleton instance of the IntakeRollers.
	 *
	 * @return The singleton instance.
	 */
	public static IntakeRollers getInstance() {
		if (mInstance == null) {
			mInstance = new IntakeRollers();
		}
		return mInstance;
	}

	public enum State {
		IDLE(0.0),
		INTAKING_CORAL(-10.0),
		INTAKING_ALGAE(8.0),
		SHOOTING_ALGAE(-8),
		EXHAUST(6.0);

		public double roller_voltage;

		State(double roller_voltage) {
			this.roller_voltage = roller_voltage;
		}
	}

	private final TalonFX mRoller;

	private State mState = State.IDLE;
	private IntakeRollerInputsAutoLogged mIntakeRollerInputs = new IntakeRollerInputsAutoLogged();
	private IntakeRollerOutputsAutoLogged mIntakeRollerOutputs = new IntakeRollerOutputsAutoLogged();

	/**
	 * Private constructor for the IntakeRollers subsystem.
	 */
	private IntakeRollers() {
		mRoller = new TalonFX(Ports.INTAKE_ROLLER.getDeviceNumber(), Ports.INTAKE_ROLLER.getBus());
		TalonUtil.applyAndCheckConfiguration(mRoller, IntakeRollerConstants.RollerFXConfig());
	}

	/**
	 * Registers the enabled loops for the subsystem.
	 *
	 * @param enabledLooper The looper to register.
	 */
	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {
			}

			@Override
			public void onLoop(double timestamp) {
				mIntakeRollerOutputs.roller_demand = mState.roller_voltage;
			}
		});
	}

	/**
	 * Gets the current state of the intake rollers.
	 *
	 * @return The current state.
	 */
	public State getState() {
		return mState;
	}

	/**
	 * Sets the state of the intake rollers.
	 *
	 * @param state The state to set.
	 */
	public void setState(State state) {
		mState = state;
	}

	/**
	 * Creates a new request to update the intake rollers with the wanted state.
	 *
	 * @param _wantedState Wanted state for the intake rollers.
	 * @return New request that updates the intake rollers with the wanted state.
	 */
	public Request stateRequest(State _wantedState) {
		return new Request() {
			@Override
			public void act() {
				setState(_wantedState);
			}

			@Override
			public boolean isFinished() {
				return mIntakeRollerOutputs.roller_demand == _wantedState.roller_voltage;
			}
		};
	}
	public Request waitForStuckRequest(){
		return new Request() {
			TimeDelayedBoolean timeout = new TimeDelayedBoolean();

			@Override
			public void act() {
			}
			@Override
			public boolean isFinished() {
				return timeout.update(mIntakeRollerInputs.roller_stator_current>75, 1);
			}
			
		};
	}

	@AutoLog
	public static class IntakeRollerInputs implements Sendable {
		// INPUTS
		public double roller_output_voltage;
		public double roller_stator_current;
		public double roller_velocity;

		@Override
		public void initSendable(SendableBuilder builder) {
			builder.addDoubleProperty("OutputVoltage", () -> roller_output_voltage, null);
			builder.addDoubleProperty("StatorCurrent", () -> roller_stator_current, null);
			builder.addDoubleProperty("VelocityRpS", () -> roller_velocity, null);
		}
	}

	@AutoLog
	public static class IntakeRollerOutputs implements Sendable {
		// OUTPUTS
		public double roller_demand;

		@Override
		public void initSendable(SendableBuilder builder) {
			builder.addDoubleProperty("Demand", () -> roller_demand, null);
		}
	}

	@Override
	public void readPeriodicInputs() {
		mIntakeRollerInputs.roller_output_voltage = mRoller.getMotorVoltage().getValue().in(Volts);
		mIntakeRollerInputs.roller_stator_current = mRoller.getStatorCurrent().getValue().in(Amps);
		mIntakeRollerInputs.roller_velocity = mRoller.getVelocity().getValue().in(RotationsPerSecond);

		Logger.processInputs("IntakeRollers", mIntakeRollerInputs);
	}

	@Override
	public void writePeriodicOutputs() {
		mRoller.setControl(new VoltageOut(mIntakeRollerOutputs.roller_demand));
	}

	@Override
	public void stop() {
		mIntakeRollerOutputs.roller_demand = 0.0;
	}

	@Override
	public boolean checkSystem() {
		return false;
	}

	@Override
	public void outputTelemetry() {

	}
}