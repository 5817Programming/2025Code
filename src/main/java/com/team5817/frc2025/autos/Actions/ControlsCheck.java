package com.team5817.frc2025.autos.Actions;

import java.util.EnumMap;
import java.util.List;

import com.team254.lib.geometry.Translation2d;
import com.team254.lib.swerve.ChassisSpeeds;
import com.team5817.frc2025.controlboard.ControlBoard;
import com.team5817.frc2025.controlboard.DriverControls;
import com.team5817.frc2025.subsystems.Superstructure;
import com.team5817.frc2025.subsystems.Drive.Drive;
import com.team5817.lib.Util;
import com.team5817.frc2025.subsystems.Superstructure.GoalState;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ControlsCheck implements Action {

  public final EnumMap<GoalState, Boolean> mGoalMap = new EnumMap<>(Superstructure.GoalState.class);

  public static List<String> statesToCheck = List.of(
      "GROUND_CORAL_INTAKE",
      "A2",
      "L4",
      "STOW",
      "CLIMB_PREPARE",
      "CLIMB_PULL",
      "NET",
      "PROCESS");

  public Superstructure s;
  public DriverControls controls;
  public ControlBoard controlBoard;
  public Drive d;

  public boolean goalsDone;
  public boolean swerveChecked;

  public ControlsCheck(Superstructure superstructure, Drive drive) {
    this.s = superstructure;
    this.d = drive;
  }

  @Override
  public void done() {
    controls = null;
  }

  @Override
  public void start() {
    controlBoard = new ControlBoard(d);
    controls = new DriverControls(d, s);

    for (GoalState goal : GoalState.values()) {
      if (statesToCheck.contains(goal.toString()))
        mGoalMap.put(goal, false);
    }

  }

  @Override
  public void update() {
    controlBoard.update();
    controls.twoControllerMode();

    d.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
        controlBoard.getSwerveTranslation().x(),
        controlBoard.getSwerveTranslation().y(),
        controlBoard.getSwerveRotation(),
        Util.robotToFieldRelative(d.getHeading(), DriverStation.getAlliance().get().equals(Alliance.Red))));

    if (new Translation2d(controlBoard.getSwerveTranslation().x(), controlBoard.getSwerveTranslation().y()).norm() > .5)
      swerveChecked = true;

    var currentGoal = s.getGoalState();
    if (statesToCheck.contains(currentGoal.toString())) {
      if (mGoalMap.get(currentGoal) != true) {
        mGoalMap.put(currentGoal, true);
      }
    }
    for (GoalState goal : mGoalMap.keySet()) {
      SmartDashboard.putBoolean("TestRoutine/" + goal.toString(), mGoalMap.get(goal));
    }
  }

  @Override
  public boolean isFinished() {
    goalsDone = true;
    for (boolean value : mGoalMap.values()) {
      if (!value) {
        goalsDone = false;
        break;
      }
    }
    return goalsDone && swerveChecked;
  }
}
