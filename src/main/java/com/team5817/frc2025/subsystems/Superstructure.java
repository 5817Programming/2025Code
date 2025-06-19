package com.team5817.frc2025.subsystems;

import com.team5817.frc2025.field.AlignmentPoint.AlignmentType;
import com.team5817.frc2025.field.FieldConstants.ReefLevel;
import com.team5817.frc2025.loops.ILooper;
import com.team5817.frc2025.loops.Loop;
import com.team5817.frc2025.subsystems.Drive.Drive;
import com.team5817.frc2025.subsystems.Elevator.Elevator;
import com.team5817.frc2025.subsystems.Elevator.ElevatorConstants;
import com.team5817.frc2025.subsystems.EndEffector.EndEffectorConstants;
import com.team5817.frc2025.subsystems.EndEffector.EndEffectorWrist;

import com.team5817.frc2025.subsystems.Intake.Intake; // Added missing import for Intake
import com.team5817.frc2025.subsystems.Rollers.RollerSubsystem;
import com.team5817.lib.drivers.Subsystem;
import com.team5817.lib.requests.ParallelRequest;
import com.team5817.lib.requests.Request;
import com.team5817.lib.requests.SequentialRequest;
import com.team5817.lib.requests.WaitRequest;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

public class Superstructure extends Subsystem {

  // Request tracking variables
  private Request activeRequest = null;
  private ArrayList<Request> queuedRequests = new ArrayList<>(0);
  private boolean hasNewRequest = false;
  private boolean allRequestsComplete = false;
  private boolean readyToScore = true;
  private boolean driverAllowsPoseComp = true;

  // Subsystems
  public Drive mDrive;
  private GoalState mGoal = GoalState.STOW;
  public RollerSubsystem<EndEffectorConstants.RollerState> mEndEffectorRollers;
  public Elevator mElevator;
  public EndEffectorWrist mEndEffectorWrist;
  public Intake mIntake;

  public enum GameObject {
    CORAL,
    ALGAE
  }

  public enum GoalState {
    STOW(new SuperstructureState(Elevator.State.STOW, EndEffectorWrist.State.STOW,
        EndEffectorConstants.RollerState.IDLE, Intake.State.IDLE,
        SuperstructureState.Type.IDLE)),
    ASTOW(new SuperstructureState(Elevator.State.STOW, EndEffectorWrist.State.STOW,
        EndEffectorConstants.RollerState.HOLD, Intake.State.IDLE,
        SuperstructureState.Type.IDLE)),
    ZERO(new SuperstructureState(Elevator.State.ZERO, EndEffectorWrist.State.ZERO,
        EndEffectorConstants.RollerState.IDLE, Intake.State.IDLE,
        SuperstructureState.Type.IDLE)),
    CLEAR(new SuperstructureState(Elevator.State.CLEAR, EndEffectorWrist.State.STOW,
        EndEffectorConstants.RollerState.IDLE, Intake.State.IDLE,
        SuperstructureState.Type.IDLE)),
    EXHAUST(new SuperstructureState(Elevator.State.STOW, EndEffectorWrist.State.STOW,
        EndEffectorConstants.RollerState.IDLE, Intake.State.EXHAUSTING,
        SuperstructureState.Type.IDLE)),
    L1(new SuperstructureState(Elevator.State.L1, EndEffectorWrist.State.L1,
        EndEffectorConstants.RollerState.l1, Intake.State.IDLE_EXAUST,
        SuperstructureState.Type.SCORING, AlignmentType.CORAL_SCORE, ReefLevel.L1)),
    L2(new SuperstructureState(Elevator.State.L2, EndEffectorWrist.State.L2,
        EndEffectorConstants.RollerState.l2, Intake.State.IDLE_EXAUST,
        SuperstructureState.Type.SCORING, AlignmentType.CORAL_SCORE, ReefLevel.L2)),
    L3(new SuperstructureState(Elevator.State.L3, EndEffectorWrist.State.L3,
        EndEffectorConstants.RollerState.l3, Intake.State.IDLE_EXAUST,
        SuperstructureState.Type.SCORING, AlignmentType.CORAL_SCORE, ReefLevel.L3)),
    L4(new SuperstructureState(Elevator.State.L4, EndEffectorWrist.State.L4,
        EndEffectorConstants.RollerState.l4, Intake.State.IDLE_EXAUST,
        SuperstructureState.Type.SCORING, AlignmentType.CORAL_SCORE, ReefLevel.L4)),
    NET(new SuperstructureState(Elevator.State.NET, EndEffectorWrist.State.STOW,
        EndEffectorConstants.RollerState.ALGAE_OUTTAKE, Intake.State.IDLE,
        SuperstructureState.Type.NET)),
    SUPER_NET(new SuperstructureState(Elevator.State.NET, EndEffectorWrist.State.STOW,
        EndEffectorConstants.RollerState.ALGAE_SHOOT, Intake.State.IDLE,
        SuperstructureState.Type.NET)),
    PROCESS(new SuperstructureState(Elevator.State.PROCESS, EndEffectorWrist.State.STOW,
        EndEffectorConstants.RollerState.ALGAE_OUTTAKE, Intake.State.IDLE,
        SuperstructureState.Type.SCORING, AlignmentType.NONE)),
    A1(new SuperstructureState(Elevator.State.A1, EndEffectorWrist.State.A1,
        EndEffectorConstants.RollerState.ALGAE_INTAKE, Intake.State.IDLE,
        SuperstructureState.Type.CLEAN, AlignmentType.ALGAE_CLEAN)),
    A2(new SuperstructureState(Elevator.State.A2, EndEffectorWrist.State.A2,
        EndEffectorConstants.RollerState.ALGAE_INTAKE, Intake.State.IDLE,
        SuperstructureState.Type.CLEAN, AlignmentType.ALGAE_CLEAN)),
    GROUND_CORAL_INTAKE(new SuperstructureState(Elevator.State.STOW, EndEffectorWrist.State.INTAKING,
        EndEffectorConstants.RollerState.CORAL_INTAKE, Intake.State.INTAKING,
        SuperstructureState.Type.INTAKING)),
    HUMAN_CORAL_INTAKE(new SuperstructureState(Elevator.State.STOW, EndEffectorWrist.State.INTAKING,
        EndEffectorConstants.RollerState.CORAL_INTAKE, Intake.State.INTAKING,
        SuperstructureState.Type.INTAKING));

    public SuperstructureState goal;

    GoalState(SuperstructureState state_goal) {
      this.goal = state_goal;
    }
  }

  /**
   * Constructor for the Superstructure class.
   */
  public Superstructure(Drive drive, Elevator elevator, EndEffectorWrist endEffectorWrist,
      RollerSubsystem<EndEffectorConstants.RollerState> endEffectorRollers,
      Intake intake) {
    mDrive = drive;
    mElevator = elevator;
    mEndEffectorWrist = endEffectorWrist;
    mEndEffectorRollers = endEffectorRollers;
    mIntake = intake;
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
        if (DriverStation.isEnabled() && DriverStation.isTeleopEnabled()) {
          double dist = driverAllowsPoseComp ? (-mDrive.getAutoAlignError().x()) : 0;
          mElevator.updateOnBranchDistance(dist);
          mEndEffectorWrist.updateOnBranchDistance(dist);
        } else {
          mElevator.updateOnBranchDistance(-1);
          mEndEffectorWrist.updateOnBranchDistance(-1);
        }
      }
    });
  }

  public void manageRequests() {
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
          r = netRequest(goal.goal);
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
            mIntake.stateRequest(goal.mIntakeState),
            mEndEffectorRollers.stateRequest(goal.mEndEffectorRollersState),
            mEndEffectorWrist.stateRequest(goal.mEndEffectorWristState)))
        .addName("Clean");
  }

  private Request IdleRequest(SuperstructureState goal) {
    if (!(goal.mType == SuperstructureState.Type.IDLE)) {
      System.out.println("Wrong Goal Type");
      return new ParallelRequest();
    }
    return new SequentialRequest(
        new ParallelRequest(
            new SequentialRequest(
                mEndEffectorWrist.stateRequest(goal.mEndEffectorWristState),
                new WaitRequest(0.1),
                mElevator.stateRequest(goal.mElevatorState)),
            mEndEffectorRollers.stateRequest(EndEffectorConstants.RollerState.HOLD),
            mIntake.stateRequest(goal.mIntakeState)),
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
        mIntake.stateRequest(Intake.State.HALF_INTAKING), // idle indexer
        new ParallelRequest(
            mEndEffectorWrist.stateRequest(goal.mEndEffectorWristState),
            mElevator.stateRequest(goal.mElevatorState),
            mEndEffectorRollers.stateRequest(goal.mEndEffectorRollersState)),
        mIntake.stateRequest(goal.mIntakeState)).addName("Intaking");
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
            mElevator.stateRequest(goal.mElevatorState),
            mIntake.stateRequest(goal.mIntakeState),
            mEndEffectorRollers.stateRequest(EndEffectorConstants.RollerState.HOLDCORAL),
            new SequentialRequest(
                mElevator.waitToBeOverRequest(ElevatorConstants.kCoralClearHeight),
                mEndEffectorWrist.stateRequest(goal.mEndEffectorWristState))),
        new SequentialRequest(
            ReadyToScoreRequest(),
            mEndEffectorRollers.stateRequest(goal.mEndEffectorRollersState)))
        .addName("Score");
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
            mElevator.stateRequest(goal.mElevatorState),
            mIntake.stateRequest(goal.mIntakeState)),
        ReadyToScoreRequest(),
        new ParallelRequest(
            mEndEffectorWrist.stateRequest(EndEffectorWrist.State.NET),
            mEndEffectorRollers.stateRequest(goal.mEndEffectorRollersState)))
        .addName("Score");
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
    }).addName("Driver Wait");
  }

  public void setReadyToScore(boolean newReady) {
    readyToScore = newReady;
  }

  public void toggleAllowPoseComp() {
    driverAllowsPoseComp = !driverAllowsPoseComp;
  }

  public Request BooleanWaitRequest(BooleanSupplier booleanSupplier, boolean target) {
    return new Request() {
      @Override
      public void act() {
      }

      @Override
      public boolean isFinished() {
        return target ? booleanSupplier.getAsBoolean() : !booleanSupplier.getAsBoolean();
      }
    };
  }
}
