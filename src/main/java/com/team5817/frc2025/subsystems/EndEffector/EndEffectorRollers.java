package com.team5817.frc2025.subsystems.EndEffector;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import javax.xml.crypto.dsig.keyinfo.RetrievalMethod;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team254.lib.drivers.TalonUtil;
import com.team254.lib.util.TimeDelayedBoolean;
import com.team5817.frc2025.Constants.EndEffectorRollerConstants;
import com.team5817.frc2025.Ports;
import com.team5817.frc2025.loops.ILooper;
import com.team5817.frc2025.loops.Loop;
import com.team5817.lib.drivers.Subsystem;
import com.team5817.lib.requests.Request;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;

/**
 * The EndEffectorRollers subsystem controls the rollers of the end effector.
 */
public class EndEffectorRollers extends Subsystem {
	private static EndEffectorRollers mInstance;

	/**
	 * Gets the singleton instance of the EndEffectorRollers subsystem.
	 *
	 * @return The singleton instance.
	 */
	public static EndEffectorRollers getInstance() {
		if (mInstance == null) {
			mInstance = new EndEffectorRollers();
		}
		return mInstance;
	}
	private double roller_demand = 0;
	public enum State {
		IDLE(0.0),
		HOLD(-2.0),
		HOLDCORAL(-1.5),
		CORAL_INTAKE(-4.0),
		l4(12.0),
		l3(3.0),
		l2(6.0),
		l1(1.5),
		ALGAE_INTAKE(-9.0),
		ALGAE_OUTTAKE(1.25),
		ALGAE_SHOOT(12);

		public double roller_voltage;

		State(double roller_voltage) {
			this.roller_voltage = roller_voltage;
		}
	}

	private final TalonFX mRoller;

	private State mState = State.IDLE;
	private EndEffectorRollerInputsAutoLogged mEndEffectorRollerInputs = new EndEffectorRollerInputsAutoLogged();

	/**
	 * Private constructor for the EndEffectorRollers subsystem.
	 */
	private EndEffectorRollers() {
		mRoller = new TalonFX(Ports.ENDEFFECTOR_ROLLER.getDeviceNumber(), Ports.ENDEFFECTOR_ROLLER.getBus());
		TalonUtil.applyAndCheckConfiguration(mRoller, EndEffectorRollerConstants.RollerFXConfig());
	}

	/**
	 * Registers the enabled loops for the subsystem.
	 *
	 * @param enabledLooper The enabled looper.
	 */
	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {
			}

			@Override
			public void onLoop(double timestamp) {
				roller_demand = mState.roller_voltage;
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

	TimeDelayedBoolean mHasPieceManager = new TimeDelayedBoolean();
	boolean hasPiece = false;
	boolean gotPiece = false;
	public boolean gotPiece(){
		return gotPiece;
	}

	/**
	 * Creates a new request that updates the intake rollers with the wanted state.
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
				return roller_demand == _wantedState.roller_voltage;
			}
		};
	}

	/**
	 * Creates a new request that sets the intake rollers to the idle state.
	 *
	 * @return New request that sets the intake rollers to the idle state.
	 */
	public Request idleRequest() {
		return new Request() {
			@Override
			public void act() {
				setState(State.IDLE);
			}

			@Override
			public boolean isFinished() {
				return roller_demand == State.IDLE.roller_voltage;
			}
		};
	}

	@AutoLog
	public static class EndEffectorRollerInputs implements Sendable {
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



	boolean lastHas;
	@Override
	public void readPeriodicInputs() {
		mEndEffectorRollerInputs.roller_output_voltage = mRoller.getMotorVoltage().getValue().in(Volts);
		mEndEffectorRollerInputs.roller_stator_current = mRoller.getStatorCurrent().getValue().in(Amps);
		mEndEffectorRollerInputs.roller_velocity = mRoller.getVelocity().getValue().in(RotationsPerSecond);

		boolean hasIntaked = mHasPieceManager.update(mEndEffectorRollerInputs.roller_stator_current>60, 0.1);
		hasPiece = hasIntaked;
		gotPiece = hasPiece!=lastHas&&hasPiece;
		lastHas = hasPiece;
		
		Logger.processInputs("EndEffectorRollers", mEndEffectorRollerInputs);
	}

	@Override
	public void writePeriodicOutputs() {
		mRoller.setControl(new VoltageOut(roller_demand));
	}

	@Override
	public void stop() {
		roller_demand = 0.0;
	}

	@Override
	public boolean checkSystem() {
		return false;
	}

	@Override
	public void outputTelemetry() {
		Logger.recordOutput("EndEffectorRollers/hasPiece", hasPiece);
	}
}