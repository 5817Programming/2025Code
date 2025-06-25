package com.team5817.frc2025.autos.Modes;

import java.util.List;

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.team254.lib.geometry.Pose2d;
import com.team5817.frc2025.autos.AutoBase;
import com.team5817.frc2025.autos.AutoConstants;
import com.team5817.frc2025.autos.AutoModeFactory.PickupLocation;
import com.team5817.frc2025.autos.AutoModeFactory.ScoringLocation;
import com.team5817.frc2025.autos.AutoModeFactory.StartingPosition;
import com.team5817.frc2025.autos.TrajectoryLibrary.l;
import com.team5817.frc2025.autos.Actions.LambdaAction;
import com.team5817.frc2025.autos.Actions.ParallelAction;
import com.team5817.frc2025.autos.Actions.SequentialAction;
import com.team5817.frc2025.autos.Actions.TrajectoryAction;
import com.team5817.frc2025.autos.Actions.TrackingTrajectoryAction;
import com.team5817.frc2025.autos.Actions.WaitAction;
import com.team5817.frc2025.autos.Actions.WaitForSuperstructureAction;
import com.team5817.frc2025.autos.Actions.WaitToPassDistanceToReef;
import com.team5817.frc2025.field.AlignmentPoint.AlignmentType;
import com.team5817.frc2025.subsystems.Superstructure;
import com.team5817.frc2025.subsystems.Superstructure.GoalState;
import com.team5817.frc2025.subsystems.Drive.Drive;
import com.team5817.frc2025.subsystems.GamePieceVision.GamepieceVision;
import com.team5817.lib.motion.Trajectory;
import com.team5817.lib.motion.TrajectorySet;

import com.team5817.frc2025.autos.Actions.Action;
import com.team5817.frc2025.autos.Actions.AutoAlignAction;
import com.team5817.frc2025.autos.Actions.EmptyAction;

import edu.wpi.first.wpilibj.Timer;

/**
 * CustomThreeCoralMode is an autonomous mode for handling three coral scoring
 * actions.
 */
public class CustomMode extends AutoBase {

  private final Drive d;
  private final Superstructure s;
  private final GamepieceVision g;
  private final TrajectorySet t;
  private int coral_amount = 0;

  private final String firstScoreName;
  private final String secondScoreName;
  private final String thirdScoreName;

  private final PickupLocation firstPickup;
  private final PickupLocation secondPickup;
  private final PickupLocation thirdPickup;

  /**
   * Constructs a CustomThreeCoralMode with the specified starting position,
   * pickup location, and scoring locations.
   *
   * @param startingPosition the starting position of the robot
   * @param thirdPickup      the location to pick up corals
   * @param firstScore       the first scoring location
   * @param secondScore      the second scoring location
   * @param thirdScore       the third scoring location
   */
  public CustomMode(StartingPosition startingPosition, PickupLocation firstPickup, PickupLocation secondPickup,
      PickupLocation thirdPickup, ScoringLocation firstScore, ScoringLocation secondScore, ScoringLocation thirdScore,
      int scoreAmount, Superstructure s, Drive d, GamepieceVision g) {
    this.s = s;
    this.d = d;
    this.g = g;

    boolean mirror = startingPosition.mirrored;
    Trajectory startingTrajectory;
    Trajectory firstPickupTrajectory;
    Trajectory secondScoreTrajectory;
    Trajectory secondPickupTrajectory;
    Trajectory thirdScoreTrajectory;
    Trajectory thirdPickupTrajectory;

    firstScoreName = firstScore.toString();
    secondScoreName = secondScore.toString();
    thirdScoreName = thirdScore.toString();

    startingTrajectory = l.trajectories.get(startingPosition.name + "To" + firstScoreName);
    firstPickupTrajectory = l.trajectories.get(firstScoreName + "To" + firstPickup.name);

    secondScoreTrajectory = l.trajectories.get(firstPickup.name + "To" + secondScoreName);
    secondPickupTrajectory = l.trajectories.get(secondScoreName + "To" + secondPickup.name);

    thirdScoreTrajectory = l.trajectories.get(secondPickup.name + "To" + thirdScoreName);
    thirdPickupTrajectory = l.trajectories.get(thirdScoreName + "To" + thirdPickup.name);

    this.firstPickup = firstPickup;
    this.secondPickup = secondPickup;
    this.thirdPickup = thirdPickup;

    this.coral_amount = scoreAmount;

    t = new TrajectorySet(
        mirror,
        startingTrajectory,
        firstPickupTrajectory,
        secondScoreTrajectory,
        secondPickupTrajectory,
        thirdScoreTrajectory,
        thirdPickupTrajectory);
  }

  /**
   * Executes the autonomous routine for scoring three corals.
   */
  @Override
  public void routine() {
    s.mElevator.setManualOffset(0.04);
    d.simResetWorldPose(t.initalPose());
    
    nextScore(firstScoreName);

    if (coral_amount == 1)
    return;

    nextIntake(firstPickup);

    System.out.println("Auto: Collected 1st coral" + " at " + (Timer.getTimestamp() - startTime));
    
    nextScoreWhenReady(secondScoreName);

    if (coral_amount == 2)
      return;

    nextIntake(secondPickup);

    System.out.println("Auto: Collected 2nd coral" + " at " + (Timer.getTimestamp() - startTime));

    nextScoreWhenReady(thirdScoreName);
    
    nextIntake(thirdPickup);
  }

  public void nextIntake(PickupLocation pickup){
    Action trajectoryAction = new EmptyAction();

    switch (pickup.state) {
      case GROUND_CORAL_INTAKE:
        trajectoryAction = new TrackingTrajectoryAction(t.next(), AutoConstants.intakeTimeout,d,g,s);
        break;
      case HUMAN_CORAL_INTAKE:
        trajectoryAction = new TrajectoryAction(t.next(), AutoConstants.intakeTimeout, d);
        break;
      default:
        System.out.println("Invalid Intake State");
        break;
    }

    r(new ParallelAction(List.of(
      trajectoryAction,
      new SequentialAction(List.of(
          new WaitToPassDistanceToReef(AutoConstants.exitDistance, d),
          new LambdaAction(() -> {
            s.setGoal(pickup.state);
          }))))));
    r(new WaitAction(AutoConstants.intakeWait));
  }

 
  public void nextScoreWhenReady(String scoreName){
    s.setReadyToScore(false);
   
    PathPlannerTrajectory traj = t.next().get().trajectory();
    edu.wpi.first.math.geometry.Pose2d pathPoses = traj.sample(traj.getTotalTimeSeconds()).pose;
    Pose2d targetPose = new Pose2d(pathPoses);

    r(new ParallelAction(List.of(
        new AutoAlignAction(d, targetPose, AlignmentType.CORAL_SCORE.tolerance),
        new SequentialAction(List.of(
            new WaitAction(AutoConstants.enterWait),
            new LambdaAction(() -> s.setGoal(GoalState.L4))
        ))
    )));

    r(new WaitAction(AutoConstants.alignWait));
    System.out.println("Auto: Starting Score " + scoreName + " at " + (Timer.getTimestamp() - startTime));
    s.setReadyToScore(true);
    r(new WaitForSuperstructureAction(s));
    r(new WaitAction(AutoConstants.coralSpit));
    
    System.out.println("Auto: Scored " + scoreName + " at " + (Timer.getTimestamp() - startTime));
    s.setReadyToScore(true);
}


  
public void nextScore(String scoreName){
    s.setReadyToScore(false);
    s.setGoal(GoalState.L4);

    PathPlannerTrajectory traj = t.next().get().trajectory();
    edu.wpi.first.math.geometry.Pose2d pathPoses = traj.sample(traj.getTotalTimeSeconds()).pose;
    Pose2d targetPose = new Pose2d(pathPoses);


    r(new AutoAlignAction(d, targetPose, AlignmentType.CORAL_SCORE.tolerance));
    r(new WaitAction(AutoConstants.alignWait));

    System.out.println("Auto: Starting Score " + scoreName + " at " + (Timer.getTimestamp() - startTime));
    s.setReadyToScore(true);

    r(new WaitForSuperstructureAction(s));
    r(new WaitAction(AutoConstants.coralSpit));
    System.out.println("Auto: Scored " + scoreName + " at " + (Timer.getTimestamp() - startTime));
}


}
