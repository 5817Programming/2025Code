package com.team5817.frc2025.autos.Modes;

import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.team5817.frc2025.autos.AutoBase;
import com.team5817.frc2025.autos.AutoConstants;
import com.team5817.frc2025.autos.Actions.LambdaAction;
import com.team5817.frc2025.autos.Actions.ParallelAction;
import com.team5817.frc2025.autos.Actions.SequentialAction;
import com.team5817.frc2025.autos.Actions.TrajectoryAction;
import com.team5817.frc2025.autos.Actions.WaitAction;
import com.team5817.frc2025.autos.Actions.WaitForBooleanAction;
import com.team5817.frc2025.autos.Actions.WaitToPassDistanceToReef;
import com.team5817.frc2025.field.AlignmentPoint.AlignmentType;
import com.team5817.frc2025.autos.AutoModeFactory.StartingPosition;
import com.team5817.frc2025.autos.Actions.WaitForSuperstructureAction;
import com.team5817.frc2025.autos.TrajectoryLibrary.l;
import com.team5817.frc2025.subsystems.Superstructure;
import com.team5817.frc2025.subsystems.Drive.Drive;
import com.team5817.frc2025.subsystems.Superstructure.GoalState;
import com.team5817.lib.motion.Trajectory;
import com.team5817.lib.motion.TrajectorySet;

import edu.wpi.first.wpilibj.Timer;

/**
 * CustomThreeCoralMode is an autonomous mode for handling three coral scoring
 * actions.
 */
public class Center11 extends AutoBase {
  private Drive d;
  private Superstructure s;
  private TrajectorySet t;

  /**
   * Constructs a CustomThreeCoralMode with the specified starting position,
   * pickup location, and scoring locations.
   *
   * @param startingPosition the starting position of the robot
   * @param pickupLocation   the location to pick up corals
   * @param firstScore       the first scoring location
   * @param secondScore      the second scoring location
   * @param thirdScore       the third scoring location
   */
  public Center11(StartingPosition startingPosition, Superstructure s, Drive d) {

    this.d = d;
    this.s = s;
    boolean mirror = startingPosition.mirrored;
    Trajectory clear1;
    Trajectory n1;
    Trajectory no;

    clear1 = l.trajectories.get("3ATo3O");
    n1 = l.trajectories.get("3CToN");
    no = l.trajectories.get("NO");

    t = new TrajectorySet(
        mirror,
        clear1,
        n1,
        no);
  }
  @Override
  public void periodic() {
    double dist = -d.getAutoAlignError().x();
    s.mElevator.updateOnBranchDistance(dist-.03);
    s.mEndEffectorWrist.updateOnBranchDistance(dist-.03);
  }
  /**
   * Executes the autonomous routine for scoring three corals.
   */
  @Override
  public void routine() {
    d.simResetWorldPose(t.initalPose());
    s.setReadyToScore(false);
    d.setAutoAlignFinishedOverride(true);
    s.request(s.AutoScoreRequest(GoalState.L4.goal, GoalState.L4.goal.mAlignmentType));
    r(new WaitAction(2));
    System.out.println("Auto:Starting Score of 3A at " + (Timer.getTimestamp() - startTime));
    s.setReadyToScore(true);

    r(new WaitForSuperstructureAction(s));
    r(new WaitAction(AutoConstants.coralSpit));

    System.out.println("Auto:Scored 3A at " + (Timer.getTimestamp() - startTime));

    r(new ParallelAction(List.of(
        new TrajectoryAction(t.next(), 0.4, d),
        new SequentialAction(new WaitToPassDistanceToReef(AutoConstants.exitDistance, d),
            new LambdaAction(() -> {
              s.setGoal(GoalState.A1);
            })))));
    d.autoAlign(AlignmentType.ALGAE_CLEAN);
    r(new WaitAction(1));
    System.out.println("Auto:Dealgaefied 6 at " + (Timer.getTimestamp() - startTime));
    scoreNet();
    System.out.println("Auto:Scored first net at " + (Timer.getTimestamp() - startTime));
    r(new TrajectoryAction(t.next(), d));
  }

  public void scoreNet() {
    s.setReadyToScore(false);
    r(new ParallelAction(List.of(
        new TrajectoryAction(t.next(), d),
        new SequentialAction(List.of(
            new WaitToPassDistanceToReef(AutoConstants.exitDistance, d),
            new LambdaAction(() -> {
              s.setGoal(GoalState.SUPER_NET);
              System.out.println("Auto:Raising at " + (Timer.getTimestamp() - startTime));
            }))))));
    // r(new WaitForBooleanAction(Elevator.getInstance()::getAtState));
    r(new WaitAction(0.2));
    s.setReadyToScore(true);
    r(new WaitAction(.4));
    s.setGoal(GoalState.STOW);
  }
}
