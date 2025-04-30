package com.team5817.frc2025.subsystems;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.TimeDelayedBoolean;
import com.team5817.frc2025.Constants;
import com.team5817.frc2025.Ports;
import com.team5817.frc2025.Robot;
import com.team5817.frc2025.field.FieldConstants;
import com.team5817.frc2025.field.FieldLayout;
import com.team5817.frc2025.field.AlignmentPoint.AlignmentType;
import com.team5817.frc2025.field.FieldConstants.ReefLevel;
import com.team5817.frc2025.loops.ILooper;
import com.team5817.frc2025.loops.Loop;
import com.team5817.frc2025.subsystems.Climb.Climb;
import com.team5817.frc2025.subsystems.Drive.AutoAlignPointSelector;
import com.team5817.frc2025.subsystems.Drive.Drive;
import com.team5817.frc2025.subsystems.Elevator.Elevator;
import com.team5817.frc2025.subsystems.EndEffector.EndEffectorRollers;
import com.team5817.frc2025.subsystems.EndEffector.EndEffectorWrist;
import com.team5817.frc2025.subsystems.EndEffector.EndEffectorRollers.State;
import com.team5817.frc2025.subsystems.Indexer.Indexer;
import com.team5817.frc2025.subsystems.Intake.IntakeDeploy;
import com.team5817.frc2025.subsystems.Intake.IntakeRollers;
import com.team5817.lib.Util;
import com.team5817.lib.drivers.BeamBreak;
import com.team5817.lib.drivers.Subsystem;
import com.team5817.lib.requests.ParallelRequest;
import com.team5817.lib.requests.Request;
import com.team5817.lib.requests.SequentialRequest;
import com.team5817.lib.requests.WaitRequest;
import com.team5817.lib.util.AllianceFlipUtil;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

public class Superstructure extends Subsystem {

	private static Superstructure mInstance;

	/**
	 * Returns the singleton instance of the Superstructure.
	 * 
	 * @return The singleton instance of the Superstructure.
	 */
	public static Superstructure getInstance() {
		if (mInstance == null) {
			mInstance = new Superstructure();
		}

		return mInstance;
	}

	// Request tracking variables
	private Request activeRequest = null;
	private ArrayList<Request> queuedRequests = new ArrayList<>(0);
	private boolean hasNewRequest = false;
	private boolean allRequestsComplete = false;
	private boolean readyToScore = true;
	private boolean driverAllowsPoseComp = true;

	// Subsystems

	// LEDs
	// private final LEDs mLEDs = LEDs.getInstance();
	// Target tracking
	public Drive mDrive;
	private double mDistanceToTarget = 0.0;
	private double mAngularErrToTarget = 0.0;
	private GoalState mGoal= GoalState.STOW;

	public Elevator mElevator;
	public EndEffectorWrist mEndEffectorWrist;
	public IntakeDeploy mIntakeDeploy;
	// public Climb mClimb;
	public EndEffectorRollers mEndEffectorRollers;
	public IntakeRollers mIntakeRollers;
	public Indexer mIndexer;

	private BeamBreak mIndexerBeam = new BeamBreak(Ports.INDEXER_BEAM_BREAK);// made it into intake
	private BeamBreak mEndEffectorBeam = new BeamBreak(Ports.END_EFFECTOR_BEAM_BREAK);// made into end effector

	public enum GameObject {
		CORAL,
		ALGAE
	}

	public enum GoalState {
		STOW(new SuperstructureState(Elevator.State.STOW, EndEffectorWrist.State.STOW, IntakeDeploy.State.DISABLE,
				Climb.State.STOW, EndEffectorRollers.State.IDLE, IntakeRollers.State.IDLE, Indexer.State.IDLE,
				SuperstructureState.Type.IDLE)),
		ASTOW(new SuperstructureState(Elevator.State.STOW, EndEffectorWrist.State.STOW, IntakeDeploy.State.DISABLE,
				Climb.State.STOW, EndEffectorRollers.State.HOLD, IntakeRollers.State.IDLE, Indexer.State.IDLE,
				SuperstructureState.Type.IDLE)),
		ZERO(new SuperstructureState(Elevator.State.ZERO, EndEffectorWrist.State.ZERO, IntakeDeploy.State.ZERO,
				Climb.State.STOW, EndEffectorRollers.State.IDLE, IntakeRollers.State.IDLE, Indexer.State.IDLE,
				SuperstructureState.Type.IDLE)),
		CLEAR(new SuperstructureState(Elevator.State.CLEAR, EndEffectorWrist.State.STOW, IntakeDeploy.State.DISABLE,
				Climb.State.STOW, EndEffectorRollers.State.IDLE, IntakeRollers.State.IDLE, Indexer.State.IDLE,
				SuperstructureState.Type.IDLE)),
		EXHAUST(new SuperstructureState(Elevator.State.STOW, EndEffectorWrist.State.STOW, IntakeDeploy.State.DISABLE,
				Climb.State.STOW, EndEffectorRollers.State.IDLE, IntakeRollers.State.EXHAUST, Indexer.State.EXHAUST,
				SuperstructureState.Type.IDLE)),	
		L1(new SuperstructureState(Elevator.State.L1, EndEffectorWrist.State.L1, IntakeDeploy.State.DISABLE,
				Climb.State.STOW, EndEffectorRollers.State.l1, IntakeRollers.State.IDLE, Indexer.State.IDLE_EXAUST,
				SuperstructureState.Type.SCORING, AlignmentType.CORAL_SCORE,ReefLevel.L1)),
		L2(new SuperstructureState(Elevator.State.L2, EndEffectorWrist.State.L2, IntakeDeploy.State.DISABLE,
				Climb.State.STOW, EndEffectorRollers.State.l2, IntakeRollers.State.IDLE, Indexer.State.IDLE_EXAUST,
				SuperstructureState.Type.SCORING, AlignmentType.CORAL_SCORE,ReefLevel.L2)),
		L2O(new SuperstructureState(Elevator.State.L2O, EndEffectorWrist.State.L2, IntakeDeploy.State.DISABLE,
				Climb.State.STOW, EndEffectorRollers.State.l2, IntakeRollers.State.IDLE, Indexer.State.IDLE_EXAUST,
				SuperstructureState.Type.SCORING, AlignmentType.CORAL_SCORE,ReefLevel.L2)),
		L3(new SuperstructureState(Elevator.State.L3, EndEffectorWrist.State.L3, IntakeDeploy.State.DISABLE,
				Climb.State.STOW, EndEffectorRollers.State.l3, IntakeRollers.State.IDLE, Indexer.State.IDLE_EXAUST,
				SuperstructureState.Type.SCORING, AlignmentType.CORAL_SCORE,ReefLevel.L3)),
		L3O(new SuperstructureState(Elevator.State.L3O, EndEffectorWrist.State.L3, IntakeDeploy.State.DISABLE,
				Climb.State.STOW, EndEffectorRollers.State.l3, IntakeRollers.State.IDLE, Indexer.State.IDLE_EXAUST,
				SuperstructureState.Type.SCORING, AlignmentType.CORAL_SCORE,ReefLevel.L3)),
		L4(new SuperstructureState(Elevator.State.L4, EndEffectorWrist.State.L4, IntakeDeploy.State.DISABLE,
				Climb.State.STOW, EndEffectorRollers.State.l4, IntakeRollers.State.IDLE, Indexer.State.IDLE_EXAUST,
				SuperstructureState.Type.SCORING, AlignmentType.CORAL_SCORE,ReefLevel.L4)),
		NET(new SuperstructureState(Elevator.State.NET, EndEffectorWrist.State.STOW, IntakeDeploy.State.DISABLE,
				Climb.State.STOW, EndEffectorRollers.State.ALGAE_OUTTAKE, IntakeRollers.State.IDLE, Indexer.State.IDLE,
				SuperstructureState.Type.NET)),
		SUPER_NET(new SuperstructureState(Elevator.State.NET, EndEffectorWrist.State.STOW, IntakeDeploy.State.DISABLE,
				Climb.State.STOW, EndEffectorRollers.State.ALGAE_SHOOT, IntakeRollers.State.IDLE, Indexer.State.IDLE,
				SuperstructureState.Type.NET)),
		PROCESS(new SuperstructureState(Elevator.State.PROCESS, EndEffectorWrist.State.STOW, IntakeDeploy.State.DISABLE,
				Climb.State.STOW, EndEffectorRollers.State.ALGAE_OUTTAKE, IntakeRollers.State.IDLE, Indexer.State.IDLE,
				SuperstructureState.Type.SCORING, AlignmentType.NONE)),
		A1(new SuperstructureState(Elevator.State.A1, EndEffectorWrist.State.A1, IntakeDeploy.State.DISABLE,
				Climb.State.STOW, EndEffectorRollers.State.ALGAE_INTAKE, IntakeRollers.State.IDLE, Indexer.State.IDLE,
				SuperstructureState.Type.CLEAN, AlignmentType.ALGAE_CLEAN)),
		A2(new SuperstructureState(Elevator.State.A2, EndEffectorWrist.State.A2, IntakeDeploy.State.DISABLE,
				Climb.State.STOW, EndEffectorRollers.State.ALGAE_INTAKE, IntakeRollers.State.IDLE, Indexer.State.IDLE,
				SuperstructureState.Type.CLEAN, AlignmentType.ALGAE_CLEAN)),
		
		GROUND_CORAL_INTAKE(new SuperstructureState(Elevator.State.STOW, EndEffectorWrist.State.INTAKING,
				IntakeDeploy.State.GROUND, Climb.State.STOW, EndEffectorRollers.State.CORAL_INTAKE,
				IntakeRollers.State.INTAKING_CORAL, Indexer.State.INDEXING, SuperstructureState.Type.INTAKING)),
		HUMAN_CORAL_INTAKE(new SuperstructureState(Elevator.State.STOW, EndEffectorWrist.State.INTAKING,
				IntakeDeploy.State.HUMAN, Climb.State.STOW, EndEffectorRollers.State.CORAL_INTAKE,
				IntakeRollers.State.INTAKING_CORAL, Indexer.State.INDEXING, SuperstructureState.Type.INTAKING)),
		CLIMB_PREPARE(new SuperstructureState(Elevator.State.STOW, EndEffectorWrist.State.STOW, IntakeDeploy.State.ZERO,
				Climb.State.PREPARE, EndEffectorRollers.State.IDLE, IntakeRollers.State.IDLE, Indexer.State.IDLE,
				SuperstructureState.Type.IDLE)),
		CLIMB_PULL(new SuperstructureState(Elevator.State.STOW, EndEffectorWrist.State.STOW, IntakeDeploy.State.ZERO,
				Climb.State.PULL, EndEffectorRollers.State.IDLE, IntakeRollers.State.IDLE, Indexer.State.IDLE,
				SuperstructureState.Type.IDLE));

		public SuperstructureState goal;

		GoalState(SuperstructureState state_goal) {
			this.goal = state_goal;
		}
	}

	/**
	 * Constructor for the Superstructure class.
	 */
	Superstructure() {
		mDrive = Drive.getInstance();
		mElevator = Elevator.getInstance();
		mEndEffectorWrist = EndEffectorWrist.getInstance();
		mIntakeDeploy = IntakeDeploy.getInstance();
		// mClimb = Climb.getInstance();
		mEndEffectorRollers = EndEffectorRollers.getInstance();
		mIntakeRollers = IntakeRollers.getInstance();
		mIndexer = Indexer.getInstance();
	}

	/**
	 * Checks if all requests have been completed.
	 * 
	 * @return True if all requests are complete, false otherwise.
	 */
	public boolean requestsCompleted() {
		return allRequestsComplete;
	}

	/**
	 * Sets a new active request and clears the request queue.
	 * 
	 * @param r The new request to be set as active.
	 */
	public void request(Request r) {
		setActiveRequest(r);
		clearRequestQueue();
	}

	/**
	 * Sets the active request.
	 * 
	 * @param request The request to be set as active.
	 */
	public void setActiveRequest(Request request) {
		activeRequest = request;
		hasNewRequest = true;
		allRequestsComplete = false;
	}

	/**
	 * Clears the request queue.
	 */
	public void clearRequestQueue() {
		queuedRequests.clear();
	}

	/**
	 * Sets the request queue with a list of requests.
	 * 
	 * @param requests The list of requests to be added to the queue.
	 */
	public void setRequestQueue(List<Request> requests) {
		clearRequestQueue();
		for (Request req : requests) {
			queuedRequests.add(req);
		}
	}

	/**
	 * Sets the request queue with an active request and a list of requests.
	 * 
	 * @param activeRequest The active request to be set.
	 * @param requests      The list of requests to be added to the queue.
	 */
	public void setRequestQueue(Request activeRequest, ArrayList<Request> requests) {
		request(activeRequest);
		setRequestQueue(requests);
	}

	/**
	 * Adds a request to the queue.
	 * 
	 * @param req The request to be added to the queue.
	 */
	public void addRequestToQueue(Request req) {
		queuedRequests.add(req);
	}
	@Override
	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {
				clearRequestQueue();
			}

			@Override
			public void onLoop(double timestamp) {
				manageRequests();
				updateLEDs();
				if(DriverStation.isEnabled()&&DriverStation.isTeleopEnabled()){
					double dist = driverAllowsPoseComp?(-mDrive.getAutoAlignError().x()):0;
					mElevator.updateBranchDistance(dist);
					mEndEffectorWrist.updateBranchDistance(dist);
					Logger.recordOutput("dist", dist);
				}else{
					mElevator.updateBranchDistance(-1);
					mEndEffectorWrist.updateBranchDistance(-1);
				}
				updateGamePieceVisualization();
			}
		});
	}
	Set<Pose3d> visualizedCoralPoses = new HashSet<>();
	public void updateGamePieceVisualization(){
		Logger.recordOutput("Coral", visualizedCoralPoses.toArray(Pose3d[]::new));
	}
	public Request visualizeScoreRequest(){
		return new Request() {
			@Override
			public void act() {
				int closestIndex = 0;
				double dist = Double.MAX_VALUE;
				for(int i = 0; i<FieldConstants.Reef.branchPositions2d.size();i++){
					double newDist = new Pose2d(AllianceFlipUtil.apply((FieldConstants.Reef.branchPositions2d.get(i).get(ReefLevel.L4)))).distance(mDrive.getPose());
					if(newDist<dist){
						dist = newDist;
						closestIndex = i;
					}
				}
				visualizedCoralPoses.add(AllianceFlipUtil.apply(FieldConstants.Reef.branchPositions.get(closestIndex).get(mGoal.goal.level)));
			}
			@Override
			public boolean isFinished() {
				return true;
			}
		};
	}
	public void manageRequests(){
		try {
			if (hasNewRequest && activeRequest != null) {
				activeRequest.act();
				hasNewRequest = false;
			}

			if (activeRequest == null) {
				if (queuedRequests.isEmpty()) {
					allRequestsComplete = true;
				} else {
					request(queuedRequests.remove(0));
				}
			} else if (activeRequest.isFinished()) {
				activeRequest = null;
			}

		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	@Override
	public void stop() {
		activeRequest = null;
		clearRequestQueue();
	}

	@Override
	public boolean checkSystem() {
		return false;
	}

	/**
	 * Gets the current goal state.
	 * 
	 * @return The current goal state.
	 */
	public GoalState getGoalState() {
		return mGoal;
	}

	@Override
	public void outputTelemetry() {
		if (activeRequest != null)
			Logger.recordOutput("State", activeRequest.getName());
		Logger.recordOutput("Elastic/AllowedPoseComp", driverAllowsPoseComp);
	}

	/* Superstructure functions */

	/**
	 * BeamBreak Sensor reading.
	 * 
	 * @param mBreak       BeamBreak Sensor.
	 * @param target_state If wanted reading is true (broken) or false (not broken).
	 * 
	 * @return Boolean for if target state is acheived.
	 */
	private Request breakWait(BeamBreak mBreak, boolean target_state) {
		return new Request() {

			@Override
			public void act() {
			}

			@Override
			public boolean isFinished() {
				if(Constants.mode==Constants.Mode.SIM){
					return true;
				}
				return mBreak.get() == target_state;
			}
		};
	}

	/**
	 * Debounced BeamBreak Sensor reading.
	 * 
	 * @param mBreak               BeamBreak Sensor.
	 * @param target_state         If wanted reading is true (broken) or false (not
	 *                             broken).
	 * @param delayed_wait_seconds Debounces time from a BeamBreak Sensor.
	 * 
	 * @return Boolean for if target state is acheived after debouncing the signal.
	 */
	private Request breakWait(BeamBreak mBreak, boolean target_state, double delayed_wait_seconds) {
		return new Request() {

			TimeDelayedBoolean timeout = new TimeDelayedBoolean();

			@Override
			public void act() {
			}

			@Override
			public boolean isFinished() {
				if(Constants.mode==Constants.Mode.SIM){
					return timeout.update(true, delayed_wait_seconds);
				}
				return timeout.update(mBreak.get() == target_state, delayed_wait_seconds);
			}
		};
	}

	/**
	 * Creates a request to wait for auto alignment to complete.
	 * 
	 * @return A request that waits for auto alignment to complete.
	 */
	private Request autoAlignWait() {
		return new Request() {
			@Override
			public void act() {
			}

			@Override
			public boolean isFinished() {
				return mDrive.getAutoAlignComplete();
			}
		}.addName("AutoAlign Wait");
	}

	/**
	 * Update state of LEDs based on BeamBreak readings.
	 */
	private void updateLEDs() {
		// switch (mLEDs.getState()) {
		// 	case INTAKING:
		// 		if (mIndexerBeam.wasTripped())
		// 			mLEDs.applyStates(TimedLEDState.INDEXING);
		// 		break;
		// 	case INDEXING:
		// 		if (mEndEffectorBeam.wasTripped())
		// 			mLEDs.applyStates(TimedLEDState.HOLDING);
		// 	default:
		// 		break;
		// }
	}


	/**
	 * Sets the goal state and creates a corresponding request.
	 * 
	 * @param goal The goal state to be set.
	 */
	public void setGoal(GoalState goal) {
		Request r = new ParallelRequest();
		if (mGoal != goal) {
			mGoal = goal;
			switch (goal.goal.mType) {
				case IDLE:
					r = IdleRequest(goal.goal);
					break;
				case SCORING:
					r = ScoreGoalRequest(goal.goal);
					break;
				case INTAKING:
					r = IntakeRequest(goal.goal);
					break;
				case CLEAN:
					r = CleanRequest(goal.goal);
					break;
				case NET:
					r= netRequest(goal.goal);
					break;
			}
			request(r);
		}
		Logger.recordOutput("Goal", goal);
	}

	/**
	 * Creates a request for cleaning based on the goal state.
	 * 
	 * @param goal The goal state.
	 * @return A request for cleaning.
	 */
	private Request CleanRequest(SuperstructureState goal) {
		return new SequentialRequest(
				new ParallelRequest(
						mElevator.stateRequest(goal.mElevatorState),
						mIndexer.stateRequest(goal.mIndexerState),
						mIntakeDeploy.stateRequest(goal.mIntakeDeployState),
						// mClimb.stateRequest(goal.mClimbState),
						mIntakeRollers.stateRequest(goal.mIntakeRollersState),
						mEndEffectorRollers.stateRequest(goal.mEndEffectorRollersState),
						mEndEffectorWrist.stateRequest(goal.mEndEffectorWristState))
				).addName("Clean");
	}

	private Request IdleRequest(SuperstructureState goal) {
		if (!(goal.mType == SuperstructureState.Type.IDLE)) {
			System.out.println("Wrong Goal Type");
			return new ParallelRequest();
		}
		return new SequentialRequest(
				new ParallelRequest(// TODO might need to add indexing as part of this, bring intake up but keep
									// indexing to get in end effector
						// mLEDs.stateRequest(TimedLEDState.INTAKING),
						new SequentialRequest(
							mEndEffectorWrist.stateRequest(goal.mEndEffectorWristState),
							new WaitRequest(0.1),
							mElevator.stateRequest(goal.mElevatorState)
						),
						mEndEffectorRollers.stateRequest(EndEffectorRollers.State.HOLD),
						mIndexer.stateRequest(goal.mIndexerState),
						mIntakeDeploy.stateRequest(goal.mIntakeDeployState),
						// mClimb.stateRequest(goal.mClimbState),
						mIntakeRollers.stateRequest(goal.mIntakeRollersState)),
						mEndEffectorRollers.stateRequest(goal.mEndEffectorRollersState)
						
		).addName("Idle");
	}


	/**
	 * Creates a request for intaking based on the goal state.
	 * 
	 * @param goal The goal state.
	 * @return A request for intaking.
	 */
	private Request IntakeRequest(SuperstructureState goal) {
		if (!(goal.mType == SuperstructureState.Type.INTAKING)) {
			System.out.println("Wrong Goal Type");
			return new ParallelRequest();
		}
		return new SequentialRequest(
			mIndexer.stateRequest(Indexer.State.IDLE),
			new ParallelRequest(// TODO might need to add indexing as part of this, bring intake up but keep
									// indexing to get in end effector
						// mLEDs.stateRequest(TimedLEDState.INTAKING),
						mEndEffectorWrist.stateRequest(goal.mEndEffectorWristState),
						mElevator.stateRequest(goal.mElevatorState),
						mEndEffectorRollers.stateRequest(goal.mEndEffectorRollersState),
						mIntakeDeploy.stateRequest(goal.mIntakeDeployState),
						// mClimb.stateRequest(goal.mClimbState),
						mIntakeRollers.stateRequest(goal.mIntakeRollersState)),
			mIndexer.stateRequest(goal.mIndexerState)
		// breakWait(mIndexerBeam, true)
		).addName("Intaking");
	}

	/**
	 * Creates a request for scoring based on the goal state.
	 * 
	 * @param goal The goal state.
	 * @return A request for scoring.
	 */
	private Request ScoreGoalRequest(SuperstructureState goal) {
		if (!(goal.mType == SuperstructureState.Type.SCORING)) {
			System.out.println("Wrong Goal Type");
		}
		return new ParallelRequest(
				new ParallelRequest(
						// mLEDs.stateRequest(TimedLEDState.PREPARING),
						mElevator.stateRequest(goal.mElevatorState),
						mIndexer.stateRequest(goal.mIndexerState),
						mIntakeDeploy.stateRequest(goal.mIntakeDeployState),
						// mClimb.stateRequest(goal.mClimbState),
						mIntakeRollers.stateRequest(goal.mIntakeRollersState),
						mEndEffectorRollers.stateRequest(EndEffectorRollers.State.HOLDCORAL),
						new SequentialRequest(
								mElevator.waitForExtensionRequest(Constants.ElevatorConstants.kCoralClearHeight),
								mEndEffectorWrist.stateRequest(goal.mEndEffectorWristState))),
				new SequentialRequest(
				ReadyToScoreRequest(),
				mEndEffectorRollers.stateRequest(goal.mEndEffectorRollersState),
				visualizeScoreRequest()
				)
				// breakWait(mEndEffectorBeam, false)
				// mLEDs.stateRequest(TimedLEDState.IDLE)
		).addName("Score");
	}
/**
	 * Creates a request for scoring based on the goal state.
	 * 
	 * @param goal The goal state.
	 * @return A request for scoring.
	 */
	private Request netRequest(SuperstructureState goal) {
		if (!(goal.mType == SuperstructureState.Type.NET)) {
			System.out.println("Wrong Goal Type");
		}
		return new SequentialRequest(
				new ParallelRequest(
						// mLEDs.stateRequest(TimedLEDState.PREPARING),
						mElevator.stateRequest(goal.mElevatorState),
						mIndexer.stateRequest(goal.mIndexerState),
						mIntakeDeploy.stateRequest(goal.mIntakeDeployState),
						// mClimb.stateRequest(goal.mClimbState),
						mIntakeRollers.stateRequest(goal.mIntakeRollersState)),
				// autoAlignWa.Homeit(),
				// mLEDs.stateRequest(TimedLEDState.PREPARED),
				ReadyToScoreRequest(),
				new ParallelRequest(
				mEndEffectorWrist.stateRequest(EndEffectorWrist.State.NET),
				mEndEffectorRollers.stateRequest(goal.mEndEffectorRollersState))
				// breakWait(mEndEffectorBeam, false)
				// mLEDs.stateRequest(TimedLEDState.IDLE)
		).addName("Score");
	}


	/**
	 * Creates a request to wait until the system is ready to score.
	 * 
	 * @return A request to wait until the system is ready to score.
	 */
	private Request ReadyToScoreRequest() {
		return (new Request() {
			@Override
			public void act() {
			}

			@Override
			public boolean isFinished() {
				return readyToScore;
			}
		}).addName("Driver Score Wait");
	}


	public void setReadyToScore(boolean newReady){
		readyToScore = newReady;
	}
	public void toggleAllowPoseComp(){
		driverAllowsPoseComp = !driverAllowsPoseComp;
	}

	/**
	 * Determines the appropriate goal state for algae cleaning based on the current
	 * position.
	 * 
	 * @return The goal state for algae cleaning.
	 */
	public GoalState AlgaeSmartCleanRequest(boolean swap) {
		boolean thing = isAlgaeHigh();
		if (swap)
			thing = !thing;
		return  thing ? GoalState.A2 : GoalState.A1;
	}
	public Request BooleanWaitRequest(BooleanSupplier booleanSupplier,boolean target){
		return new Request() {
			@Override
			public void act() {
			}
			@Override
			public boolean isFinished() {
				return target?booleanSupplier.getAsBoolean():!booleanSupplier.getAsBoolean();
			}
		};
	}

	/**
	 * Determines if the algae is high based on the current position.
	 * 
	 * @return True if the algae is high, false otherwise.
	 */
	private boolean isAlgaeHigh() {
		Translation2d reef_to_odom = FieldLayout.getReefPose().inverse().translateBy(mDrive.getPose().getTranslation());
		double angle = Math.toDegrees(Math.atan2(reef_to_odom.x(), reef_to_odom.y())) + 30;
		boolean out = Math.floorMod(Math.round(angle / 60), 2) == 0;
		return Util.isRed().get()?out:!out;
	}
}