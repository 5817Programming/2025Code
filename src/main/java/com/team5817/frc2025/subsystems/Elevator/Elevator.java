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

	private State mState = State.ZERO;
	private boolean atState = false;
	private double branchDist = 0;
	private double offset = 0;
	

	final static double kStrictError = .05;
	final static double kMediumError = .1;
	final static double kLenientError = .2;

	/**
	 * Enum representing the different states of the elevator.
	 */
	public enum State {
		L4(1.8635330123363545, kStrictError, ElevatorConstants.kHighOffsetMap),
		L3(1.211806509200769-.06, kStrictError, ElevatorConstants.kMidOffsetMap),
		L2(.754804+.03, kStrictError, ElevatorConstants.kMidOffsetMap),
		L1(0.219, kStrictError),
		A1(0.59, kMediumError),
		A2(1, kMediumError),
		NET(2.035, kStrictError),
		ZERO(0, kLenientError),
		PROCESS(0.0, kLenientError),
		CLEAR(.35,kStrictError),
		STOW(0.0, kStrictError);

		double output = 0;
		double allowable_error = 20;
		InterpolatingDoubleTreeMap map;

		
		State(double output, double allowable_error) {
			this(output, allowable_error, null);
		}
		State(double output, double allowable_error, InterpolatingDoubleTreeMap map) {
			this.output = output;
			this.allowable_error = allowable_error;
			this.map = map;
		}


		public double getTrackedOutput(double position){
			if(map == null){
				return output;
			}
			double des = this.output + map.get(position);
			return des;
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
			}
		});
	}
	public void updateBranchDistance(double dist){
		this.branchDist = dist;
	}
	public void setManualOffset(double offset){
		this.offset = offset;
	}
	public void changeManualOffset(double deltaOffset){
		this.offset+=deltaOffset;
	}


	@Override
	public void readPeriodicInputs() {
		super.readPeriodicInputs();
		Logger.processInputs("Elevator", mServoInputs);
	}

	@Override
	public void writePeriodicOutputs() {
		double trackedOutput = mState.getTrackedOutput(branchDist);
		if(mState==State.L1||mState==State.L2||mState==State.L3||mState==State.L4)
			trackedOutput+=offset;
		trackedOutput = Util.limit(trackedOutput, ElevatorConstants.kElevatorServoConstants.kMinUnitsLimit, ElevatorConstants.kElevatorServoConstants.kMaxUnitsLimit);
		atState = Util.epsilonEquals(getPosition(),trackedOutput , mState.allowable_error);
		if (mControlState == ControlState.MOTION_MAGIC)
			setSetpointMotionMagic(trackedOutput);
		super.writePeriodicOutputs();
	}

	@Override
	public void outputTelemetry() {
		Pose3d current = new Pose3d(Math.cos(Units.degreesToRadians(84)) * mServoInputs.position_units, 0,
				Math.sin(Units.degreesToRadians(84)) * mServoInputs.position_units, new Rotation3d());

		Robot.mechPoses[1] = current.div(3);
		Robot.mechPoses[2] = current.div(3).times(2);
		Robot.mechPoses[3] = current;

		Pose3d desired = new Pose3d(Math.cos(Units.degreesToRadians(84)) * demand, 0,
				Math.sin(Units.degreesToRadians(84)) * demand, new Rotation3d());

		Robot.desMechPoses[1] = desired.div(3);
		Robot.desMechPoses[2] = desired.div(3).times(2);
		Robot.desMechPoses[3] = desired;
		Logger.recordOutput("Elevator/Offset", this.offset);
		Logger.recordOutput(mConstants.kName+"/AtState", atState);
		Logger.recordOutput(mConstants.kName+"/State",	mState);
		super.outputTelemetry();
	}
	public boolean getAtState(){
		return atState;
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
				if (mControlState != ControlState.MOTION_MAGIC) {
					mControlState = ControlState.MOTION_MAGIC;
				}
				mState = _wantedState;
			}

			@Override
			public boolean isFinished() {
				return atState;
			}
		};
	}

}