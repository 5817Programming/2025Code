package com.team5817.frc2025.subsystems.Elevator;

import com.team5817.frc2025.Robot;
import com.team5817.frc2025.Constants.ElevatorConstants;
import com.team5817.frc2025.loops.ILooper;
import com.team5817.frc2025.loops.Loop;
import com.team5817.lib.Util;
import com.team5817.lib.drivers.ServoMotorSubsystem;
import com.team5817.lib.requests.Request;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

import org.littletonrobotics.junction.Logger;

/**
 * Elevator subsystem for controlling the elevator mechanism.
 */
public class Elevator extends ServoMotorSubsystem {
	public static Elevator mInstance;

	/**
	 * Returns the singleton instance of the Elevator.
	 * 
	 * @return the singleton instance of the Elevator
	 */
	public static Elevator getInstance() {
		if (mInstance == null) {
			mInstance = new Elevator(ElevatorConstants.kElevatorServoConstants);
		}
		return mInstance;
	}

	private State currentState = State.ZERO;
	private boolean atState = false;
	private double branchDist = 0;
	

	final static double kStrictError = .05;
	final static double kMediumError = .1;
	final static double kLenientError = .2;

	/**
	 * Enum representing the different states of the elevator.
	 */
	public enum State {
		L4(1.8, kStrictError, ElevatorConstants.kHighOffsetMap),
		L3(.886, kStrictError, ElevatorConstants.kMidOffsetMap),
		L2(0.533, kStrictError, ElevatorConstants.kMidOffsetMap),
		L1(0.429, kStrictError),
		A1(0.59, kMediumError),
		A2(1, kMediumError),
		NET(2, kMediumError),
		ZERO(0.009, kLenientError),
		PROCESS(0.0, kLenientError),
		CLEAR(1.9,kStrictError),
		STOW(0.0, kStrictError);

		double output = 0;
		double allowable_error = 20;
		boolean home = false;
		InterpolatingDoubleTreeMap map;

		State(double output, double allowable_error) {
			this(output, allowable_error, null,false);
		}
		State(double output, double allowable_error, boolean home) {
			this(output, allowable_error, null,home);
		}
		State(double output, double allowable_error, InterpolatingDoubleTreeMap map) {
			this(output, allowable_error, map,false);
		}
		State(double output, double allowable_error, InterpolatingDoubleTreeMap map, boolean home) {
			this.output = output;
			this.allowable_error = allowable_error;
			this.map = map;
			this.home = home;
		}


		public double getTrackedOutput(double position){
			if(map == null){
				return output;
			}
			double des = this.output + map.get(position);
			return Util.limit(des, ElevatorConstants.kElevatorServoConstants.kMinUnitsLimit, ElevatorConstants.kElevatorServoConstants.kMaxUnitsLimit);
		}
	}

	/**
	 * Constructs an Elevator with the given constants.
	 * 
	 * @param constants the constants for the elevator
	 */
	public Elevator(final ServoMotorSubsystemConstants constants) {
		super(constants);
		mMain.setPosition(homeAwareUnitsToRotations(0.0));
		enableSoftLimits(false);
		setSetpointMotionMagic(State.ZERO.output);
	}

	/**
	 * Registers the enabled loops for the elevator.
	 * 
	 * @param enabledLooper the enabled looper
	 */
	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {
			}

			@Override
			public void onLoop(double timestamp) {
				if (getSetpoint() == mConstants.kHomePosition && atHomingLocation() && mWantsHome && !mHoming) {
					setWantHome(true);
				} else if (mControlState != ControlState.OPEN_LOOP && mHoming) {
					setWantHome(false);
				}

				double trackedOutput = currentState.getTrackedOutput(branchDist);
				atState = Util.epsilonEquals(getPosition(),trackedOutput , currentState.allowable_error);
				setSetpointMotionMagic(trackedOutput);
			}
		});
	}
	public void updateBranchDistance(double dist){
		this.branchDist = dist;
	}
	@Override
	public boolean atHomingLocation() {
		return true;
		// return Util.epsilonEquals(mServoInputs.position_units, mConstants.kHomePosition,ElevatorConstants.kHomingZone);
	}


	@Override
	public void readPeriodicInputs() {
		super.readPeriodicInputs();
		Logger.processInputs("Elevator", mServoInputs);
	}

	@Override
	public void writePeriodicOutputs() {
		if (mHoming) {
			setOpenLoop(mConstants.kHomingOutput / mConstants.kMaxForwardOutput);
			if (mHomingDelay.update(
					Timer.getFPGATimestamp(),
					Math.abs(getVelocity()) < mConstants.kHomingVelocityWindow)) {
				zeroSensors();
				mHasBeenZeroed = true;
				setSetpointMotionMagic(mConstants.kHomePosition);
				mHoming = false;
			}
		}

		super.writePeriodicOutputs();
	}

	@Override
	public void outputTelemetry() {
		Pose3d current = new Pose3d(Math.cos(Units.degreesToRadians(84)) * mServoInputs.position_units, 0,
				Math.sin(Units.degreesToRadians(84)) * mServoInputs.position_units, new Rotation3d());

		Robot.mechPoses[2] = current.div(3);
		Robot.mechPoses[3] = current.div(3).times(2);
		Robot.mechPoses[4] = current;

		Pose3d desired = new Pose3d(Math.cos(Units.degreesToRadians(84)) * demand, 0,
				Math.sin(Units.degreesToRadians(84)) * demand, new Rotation3d());

		Robot.desMechPoses[2] = desired.div(3);
		Robot.desMechPoses[3] = desired.div(3).times(2);
		Robot.desMechPoses[4] = desired;

		super.outputTelemetry();
	}

	@Override
	public void stop() {
	}

	@Override
	public boolean checkSystem() {
		return false;
	}

	/**
	 * Returns a request to wait for the elevator to extend to the given position.
	 * 
	 * @param position the position to wait for
	 * @return a request to wait for the elevator to extend
	 */
	public Request waitForExtensionRequest(double position) {
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
	 * Returns a request to set the elevator to the given state.
	 * 
	 * @param _wantedState the state to set the elevator to
	 * @return a request to set the elevator to the given state
	 */
	public Request stateRequest(State _wantedState) {
		return new Request() {
			@Override
			public void act() {
				currentState = _wantedState;
				mWantsHome = _wantedState.home;
			}

			@Override
			public boolean isFinished() {
				return atState;
			}
		};
	}

}