package com.team5817.frc2025.autos.Modes;

import java.util.List;

import com.team5817.frc2025.autos.AutoBase;
import com.team5817.frc2025.autos.AutoConstants;
import com.team5817.frc2025.autos.Actions.LambdaAction;
import com.team5817.frc2025.autos.Actions.ParallelAction;
import com.team5817.frc2025.autos.Actions.SequentialAction;
import com.team5817.frc2025.autos.Actions.TrajectoryAction;
import com.team5817.frc2025.autos.Actions.WaitAction;
import com.team5817.frc2025.autos.Actions.WaitToPassDistanceToReef;
import com.team5817.frc2025.autos.AutoModeSelector.StartingPosition;
import com.team5817.frc2025.autos.Actions.WaitForSuperstructureAction;
import com.team5817.frc2025.autos.TrajectoryLibrary.l;
import com.team5817.frc2025.subsystems.Superstructure;
import com.team5817.frc2025.subsystems.Superstructure.GoalState;
import com.team5817.lib.RobotMode;
import com.team5817.lib.motion.Trajectory;
import com.team5817.lib.motion.TrajectorySet;

import edu.wpi.first.wpilibj.Timer;

/**
 * CustomThreeCoralMode is an autonomous mode for handling three coral scoring
 * actions.
 */
public class Center12 extends AutoBase {

  private Superstructure s = Superstructure.getInstance();
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
  public Center12(StartingPosition startingPosition) {

    boolean mirror = startingPosition.mirrored;
    Trajectory c1;
    Trajectory clear1;
    Trajectory da1;
    Trajectory n1;
    Trajectory da2;
    Trajectory n2;

    c1 = l.trajectories.get(startingPosition.name + "To_3A");
    clear1 = l.trajectories.get("3ATo3O");
    da1 = l.trajectories.get("3OTo3C");
    n1 = l.trajectories.get("3CToN");
    da2 = l.trajectories.get("NTo4C");
    n2 = l.trajectories.get("4CToN");

    t = new TrajectorySet(
        mirror,
        c1,
        clear1,
        da1,
        n1,
        da2,
        n2);
  }

  /**
   * Executes the autonomous routine for scoring three corals.
   */
  @Override
  public void routine() {
    if (RobotMode.isSim()) {
      mSim.setSimulationWorldPose(t.initalPose().wpi());
    }
    s.setReadyToScore(false);
    r(new ParallelAction(List.of(
        new TrajectoryAction(t.next(), 2),
        new LambdaAction(() -> {
          s.setGoal(GoalState.L4);
        }))));
    r(new WaitAction(.5));
    System.out.println("Auto:Starting Score of 3A at " + (Timer.getTimestamp() - startTime));
    s.setReadyToScore(true);

    r(new WaitForSuperstructureAction());
    r(new WaitAction(AutoConstants.coralSpit));

    System.out.println("Auto:Scored 3A at " + (Timer.getTimestamp() - startTime));

    r(new ParallelAction(List.of(
        new TrajectoryAction(t.next(), 0.4),
        new SequentialAction(new WaitToPassDistanceToReef(AutoConstants.exitDistance),
            new LambdaAction(() -> {
              s.setGoal(GoalState.A1);
            })))));
    r(new TrajectoryAction(t.next(), 1));
    System.out.println("Auto:Dealgaefied 6 at " + (Timer.getTimestamp() - startTime));
    scoreNet();
    System.out.println("Auto:Scored first net at " + (Timer.getTimestamp() - startTime));
    s.setGoal(GoalState.A2);
    r(new TrajectoryAction(t.next(), .5));
    System.out.println("Auto:Dealgaefied 5 at " + (Timer.getTimestamp() - startTime));
    scoreNet();
    System.out.println("Auto:Scored second net at " + (Timer.getTimestamp() - startTime));

  }

  public void scoreNet() {
    s.setReadyToScore(false);
    r(new ParallelAction(List.of(
        new TrajectoryAction(t.next()),
        new SequentialAction(List.of(
            new WaitToPassDistanceToReef(AutoConstants.exitDistance),
            new LambdaAction(() -> {
              s.setGoal(GoalState.SUPER_NET);
              System.out.println("Auto:Raising at " + (Timer.getTimestamp() - startTime));
            }))))));
    // r(new WaitForBooleanAction(Elevator.getInstance()::getAtState));
    r(new WaitAction(0.2));
    s.setReadyToScore(true);
    r(new WaitAction(.3));
    s.setGoal(GoalState.STOW);

  }
}
