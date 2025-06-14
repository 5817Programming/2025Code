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
public class Center11 extends AutoBase {

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
  public Center11(StartingPosition startingPosition) {

    boolean mirror = startingPosition.mirrored;
    Trajectory coral;
    Trajectory backup;
    Trajectory deAlgaefy;
    Trajectory net;
    Trajectory out;

    coral = l.trajectories.get(startingPosition.name + "To_3A");
    backup = l.trajectories.get("3ATo3O");
    deAlgaefy = l.trajectories.get("3OTo3C");
    net = l.trajectories.get("3CToN");
    out = l.trajectories.get("NToO");

    t = new TrajectorySet(
        mirror,
        coral,
        backup,
        deAlgaefy,
        net,
        out

    );
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
        new TrajectoryAction(t.next(), AutoConstants.scoreTimeout),
        new LambdaAction(() -> {
          s.setGoal(GoalState.L4);
        }))));
    r(new WaitAction(AutoConstants.alignWait));
    System.out.println("Starting Score of 3A at " + (Timer.getTimestamp() - startTime));
    s.setReadyToScore(true);

    r(new WaitForSuperstructureAction());
    r(new WaitAction(AutoConstants.coralSpit));

    System.out.println("Scored 3A at " + (Timer.getTimestamp() - startTime));

    r(new ParallelAction(List.of(
        new TrajectoryAction(t.next(), 0.4),
        new SequentialAction(new WaitToPassDistanceToReef(AutoConstants.exitDistance),
            new LambdaAction(() -> {
              s.setGoal(GoalState.A1);
            })))));
    s.setReadyToScore(false);
    r(new WaitForSuperstructureAction());
    System.out.println("Dealgaefing at " + (Timer.getTimestamp() - startTime));
    r(new TrajectoryAction(t.next(), .5));
    System.out.println("Waiting for entry at " + (Timer.getTimestamp() - startTime));
    r(new WaitAction(1));// intake wait
    s.setGoal(GoalState.A1);
    System.out.println("Dealgaefied at " + (Timer.getTimestamp() - startTime));
    r(new ParallelAction(List.of(
        new TrajectoryAction(t.next()),
        new SequentialAction(List.of(
            new WaitToPassDistanceToReef(AutoConstants.exitDistance),
            new LambdaAction(() -> {
              s.setGoal(GoalState.ASTOW);
              System.out.println("Stowed at " + (Timer.getTimestamp() - startTime));
            }))))));
    System.out.println("Raising at " + (Timer.getTimestamp() - startTime));
    s.setGoal(GoalState.NET);
    r(new WaitAction(1.5));
    System.out.println("Scoring at " + (Timer.getTimestamp() - startTime));
    s.setReadyToScore(true);
    r(new WaitAction(1));
    s.setGoal(GoalState.STOW);
    r(new WaitAction(1));
    r(new TrajectoryAction(t.next()));
  }
}
