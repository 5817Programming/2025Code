package com.team5817.frc2025.subsystems.Intake;

import com.team5817.frc2025.Constants;
import com.team5817.frc2025.Robot;
import com.team5817.frc2025.Constants.IntakeDeployConstants;
import com.team5817.frc2025.loops.ILooper;
import com.team5817.frc2025.loops.Loop;
import com.team5817.lib.Util;
import com.team5817.lib.drivers.ServoMotorSubsystemWithCancoder;
import com.team5817.lib.requests.Request;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;

/**
 * The IntakeDeploy class controls the deployment mechanism of the intake system.
 */
public class IntakeDeploy extends ServoMotorSubsystemWithCancoder {

	public static IntakeDeploy mInstance;

	/**
	 * Gets the singleton instance of the IntakeDeploy class.
	 *
	 * @return The instance of the IntakeDeploy class.
	 */
	public static IntakeDeploy getInstance() {
		if (mInstance == null) {
			mInstance = new IntakeDeploy(IntakeDeployConstants.kDeployServoConstants,
					IntakeDeployConstants.kDeployEncoderConstants);
		}
		return mInstance;
	}

	final static double kStrictError = .5;
	final static double kMediumError = 2;
	final static double kLenientError = 5;

	/**
	 * Represents the different states of the intake deployment.
	 */
	public enum State {
		DEPLOY(.245, kStrictError), // TODO
		CLEAR(0.0, kLenientError), // TODO
		UNJAM(0.0, kLenientError), // TODO
		STOW(0.0, kMediumError),
		ALGAE(0.134, kMediumError),
		HUMAN(0.0, kStrictError),
		ZERO(0.0, kStrictError);

		double output = 0;
		double allowable_error = 0;

		/**
		 * Constructs a new State.
		 *
		 * @param output The output value for the state.
		 * @param allowable_error The allowable error for the state.
		 */
		State(double output, double allowable_error) {
			this.output = output;
			this.allowable_error = allowable_error;
		}

	}


	/**
	 * Constructs a new IntakeDeploy subsystem.
	 *
	 * @param constants The constants for the servo motor subsystem.
	 * @param encoder_constants The constants for the absolute encoder.
	 */
	public IntakeDeploy(final ServoMotorSubsystemConstants constants,
			final AbsoluteEncoderConstants encoder_constants) {
		super(constants, encoder_constants);
		mMain.setPosition(homeAwareUnitsToRotations(0.0));
		enableSoftLimits(false);
		setSetpointMotionMagic(State.DEPLOY.output);
	}

	/**
	 * Registers the enabled loops for the subsystem.
	 *
	 * @param enabledLooper The looper to register the loops with.
	 */
	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {
			}

			@Override
			public void onLoop(double timestamp) {
				
			}
		});
	}

	/**
	 * Reads the periodic inputs for the subsystem.
	 */
	@Override
	public void readPeriodicInputs() {
		super.readPeriodicInputs();
		Logger.processInputs("IntakeDeploy", mServoInputs);
	}

	/**
	 * Writes the periodic outputs for the subsystem.
	 */
	@Override
	public void writePeriodicOutputs() {
		super.writePeriodicOutputs();
	}

	/**
	 * Outputs telemetry data for the subsystem.
	 */
	@Override
	public void outputTelemetry() {
		Robot.mechPoses[0] = new Pose3d(new Translation3d(-.325, 0, .261), new Rotation3d(Units.degreesToRadians(0),
				Units.rotationsToRadians(0.3 - mServoInputs.position_units), Units.degreesToRadians(0)));

		Robot.desMechPoses[0] = new Pose3d(new Translation3d(-.325, 0, .261), new Rotation3d(Units.degreesToRadians(0),
				Units.rotationsToRadians(0.3 - mServoOutputs.demand), Units.degreesToRadians(0)));

		
		super.outputTelemetry();
	}

	/**
	 * Stops the subsystem.
	 */
	@Override
	public void stop() {
	}

	/**
	 * Checks the system for any issues.
	 *
	 * @return True if the system is functioning correctly, false otherwise.
	 */
	@Override
	public boolean checkSystem() {
		return false;
	}

	/**
	 * Checks if the intake is at the homing location.
	 *
	 * @return True if the intake is within the homing zone, false otherwise.
	 */
	@Override
	public boolean atHomingLocation() {
		// Check if intake is within 7.0 degrees of homing position
		return mServoInputs.position_units - mConstants.kHomePosition > -Constants.IntakeDeployConstants.kHomingZone;
	}

	/**
	 * Creates a request to change the state of the intake deployment.
	 *
	 * @param _wantedState The desired state.
	 * @return The request to change the state.
	 */
	public Request stateRequest(State _wantedState) {
		return new Request() {
			@Override
			public void act() {
				setSetpointMotionMagic(_wantedState.output);
			}

			@Override
			public boolean isFinished() {
				return Util.epsilonEquals(getPosition(), _wantedState.output, _wantedState.allowable_error);
			}
		};
	}
}