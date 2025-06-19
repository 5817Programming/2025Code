package com.team5817.frc2025.controlboard;

import org.littletonrobotics.junction.Logger;

import com.team5817.frc2025.field.AlignmentPoint.AlignmentType;
import com.team5817.frc2025.field.FieldLayout;
import com.team5817.frc2025.subsystems.Superstructure;
import com.team5817.frc2025.subsystems.Superstructure.GoalState;
import com.team5817.lib.Util;
import com.team5817.frc2025.subsystems.Drive.Drive;
import com.team5817.frc2025.subsystems.Drive.Drive.DriveControlState;
import com.team5817.frc2025.subsystems.EndEffector.EndEffectorConstants;
import com.team5817.frc2025.subsystems.Intake.Intake;

/**
 * The DriverControls class handles the input from the driver and co-driver
 * controllers
 * and translates them into actions for the robot's subsystems.
 */
public class DriverControls {

  public ControlBoard mControlBoard;

  Superstructure s;
  Drive d;

  /**
   * Constructor for the DriverControls class.
   * Initializes the Drive and Superstructure instances and sets the initial goal
   * state.
   */
  public DriverControls(Drive d, Superstructure s) {
    this.d = d;
    this.s = s;
    this.mControlBoard = new ControlBoard(d);
    this.driver = mControlBoard.driver;
    this.codriver = mControlBoard.operator;
  }

  /* ONE CONTROLLER */

  /**
   * Handles the input for the one controller mode.
   * This mode is used when only one controller is available for the driver.
   */
  public void oneControllerMode() {
    // mDrive.overrideHeading(true);
    if (driver.getStartButton())
      d.zeroGyro();
  }

  CustomXboxController driver;
  CustomXboxController codriver;
  /* TWO CONTROLLERS */
  GoalState preparedGoal = GoalState.L4;
  double lastTime = 0;

  boolean wantStow = false;

  /**
   * Handles the input for the two controller mode.
   * This mode is used when both driver and co-driver controllers are available.
   */
  public void twoControllerMode() {
    if (driver.getStartButton())
      d.zeroGyro(Util.isRed().get() ? 0 : 180);
    if (driver.getBackButton())
      d.zeroGyro(Util.isRed().get() ? 180 : 0);

    // if(driver.\)sswaaaaaa
    if (driver.leftBumper.isBeingPressed()) {
      s.setGoal(GoalState.GROUND_CORAL_INTAKE);
    }
    if (driver.leftTrigger.isBeingPressed()) {
      s.setGoal(preparedGoal);

    } else

    if (driver.rightTrigger.isBeingPressed() && !driver.POV270.isBeingPressed() && !driver.POV90.isBeingPressed()) {
      if (s.getGoalState().goal.mAlignmentType != AlignmentType.NONE)
        d.autoAlign(s.getGoalState().goal.mAlignmentType);
      else
        d.setAutoAlignFinishedOverrride(true);
    }
    if (driver.POV270.isBeingPressed())
      d.autoAlign(AlignmentType.CORAL_SCORE_LEFT);
    if (driver.POV90.isBeingPressed())
      d.autoAlign(AlignmentType.CORAL_SCORE_RIGHT);

    if (driver.getAButton()) {
      s.setGoal(GoalState.A2);
    }
    if (driver.getXButton()) {
      s.setGoal(GoalState.A1);
    }
    if (driver.bButton.isBeingPressed())
      s.mEndEffectorRollers.setState(EndEffectorConstants.RollerState.CORAL_INTAKE);
    if (driver.yButton.isBeingPressed()) {
      s.mIntake.stateRequest(Intake.State.HALF_INTAKING).act();
    }
    // if(driver.xButton.isBeingPressed())
    // s.setGoal(GoalState.GROUND_ALGAE_INTAKE);

    if (driver.releasedAny(driver.leftBumper, driver.bButton, driver.yButton)
        || (driver.getLeftTriggerAxis() != 1 && driver.rightBumper.wasReleased())) {

      s.setGoal(GoalState.STOW);
      d.setControlState(DriveControlState.OPEN_LOOP);
    }
    if (driver.releasedAny(driver.aButton, driver.xButton))
      s.setGoal(GoalState.ASTOW);

    if (driver.releasedAny(driver.leftTrigger) && !(driver.getAButton() || driver.getXButton())) {
      wantStow = true;
    }
    if (wantStow && clearReef()) {
      s.setGoal(GoalState.STOW);

      d.setControlState(DriveControlState.OPEN_LOOP);
      wantStow = false;
    }

    if (driver.releasedAny(driver.rightTrigger))
      d.setControlState(DriveControlState.OPEN_LOOP);

    if (driver.leftTrigger.isBeingPressed())
      s.setReadyToScore(driver.rightBumper.isBeingPressed());
    else if (driver.rightBumper.isBeingPressed())
      s.setGoal(GoalState.EXHAUST);

    if (codriver.getRightTriggerAxis() == 1)
      s.mIntake.stateRequest(Intake.State.STOW).act();
    if (codriver.getLeftTriggerAxis() == 1)
      s.setGoal(GoalState.CLEAR);
    if (codriver.getStartButtonPressed())
      s.toggleAllowPoseComp();
    if (s.getGoalState() == GoalState.CLEAR && codriver.getLeftTriggerAxis() != 1)
      s.setGoal(GoalState.STOW);
    if (codriver.yButton.isBeingPressed())
      preparedGoal = GoalState.L4;
    if (codriver.bButton.isBeingPressed())
      preparedGoal = GoalState.L3;
    if (codriver.aButton.isBeingPressed())
      preparedGoal = GoalState.L2;

    if (codriver.xButton.isBeingPressed())
      preparedGoal = GoalState.L1;
    if (codriver.POV0.isBeingPressed()) {
      preparedGoal = GoalState.NET;
    }
    if (codriver.POV180.isBeingPressed()) {
      preparedGoal = GoalState.PROCESS;
    }

    if (codriver.POV270.wasReleased())
      s.mElevator.home();
    if (codriver.POV90.wasReleased())
      s.mEndEffectorWrist.home();

    Logger.recordOutput("Elastic/PreparedGoal", preparedGoal);
  }

  public boolean clearReef() {
    return d.getPose().getTranslation().translateBy(FieldLayout.getReefPose().inverse().getTranslation())
        .norm() > 1.4;
  }
}
