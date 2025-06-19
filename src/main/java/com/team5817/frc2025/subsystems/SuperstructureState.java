package com.team5817.frc2025.subsystems;

import com.team5817.frc2025.field.AlignmentPoint.AlignmentType;
import com.team5817.frc2025.field.FieldConstants.ReefLevel;
import com.team5817.frc2025.subsystems.Elevator.Elevator;
import com.team5817.frc2025.subsystems.EndEffector.EndEffectorConstants;
import com.team5817.frc2025.subsystems.EndEffector.EndEffectorWrist;
import com.team5817.frc2025.subsystems.Intake.Intake;

/**
 * Represents the state of the superstructure, including various subsystems.
 */
public class SuperstructureState {

  public final Elevator.State mElevatorState;
  public final EndEffectorWrist.State mEndEffectorWristState;
  public final EndEffectorConstants.RollerState mEndEffectorRollersState;
  public final Intake.State mIntakeState;
  public final Type mType;
  public final AlignmentType mAlignmentType;

  /**
   * Enum representing the type of superstructure state.
   */
  public enum Type {
    SCORING,
    CLEAN,
    INTAKING,
    IDLE,
    NET;

  };

  /**
   * Constructs a SuperstructureState with the specified subsystem states and
   * type.
   *
   * @param elevator_state    the state of the elevator
   * @param wrist_state       the state of the end effector wrist
   * @param climb_state       the state of the climb
   * @param endEffector_state the state of the end effector rollers
   * @param indexer_state     the state of the indexer
   * @param type              the type of the superstructure state
   */
  public SuperstructureState(
      Elevator.State elevator_state,
      EndEffectorWrist.State wrist_state,
      EndEffectorConstants.RollerState endEffector_state,
      Intake.State intake_state,
      Type type) {

    this.mElevatorState = elevator_state;
    this.mEndEffectorWristState = wrist_state;
    this.mEndEffectorRollersState = endEffector_state;
    this.mIntakeState = intake_state;
    this.mType = type;
    this.mAlignmentType = AlignmentType.NONE;

    // create into enum
  }

  /**
   * Constructs a SuperstructureState with the specified subsystem states, type,
   * and alignment type.
   *
   * @param elevator_state    the state of the elevator
   * @param wrist_state       the state of the end effector wrist
   * @param climb_state       the state of the climb
   * @param endEffector_state the state of the end effector rollers
   * @param indexer_state     the state of the indexer
   * @param type              the type of the superstructure state
   * @param alignmentTypes    the alignment type
   */
  public SuperstructureState(
      Elevator.State elevator_state,
      EndEffectorWrist.State wrist_state,
      EndEffectorConstants.RollerState endEffector_state,
      Intake.State intake_state,
      Type type, AlignmentType alignmentTypes) {

    this.mElevatorState = elevator_state;
    this.mEndEffectorWristState = wrist_state;
    this.mEndEffectorRollersState = endEffector_state;
    this.mIntakeState = intake_state;
    this.mType = type;
    this.mAlignmentType = alignmentTypes;
  }

  /**
   * Constructs a SuperstructureState with the specified subsystem states, type,
   * and alignment type.
   *
   * @param elevator_state    the state of the elevator
   * @param wrist_state       the state of the end effector wrist
   * @param climb_state       the state of the climb
   * @param endEffector_state the state of the end effector rollers
   * @param indexer_state     the state of the indexer
   * @param type              the type of the superstructure state
   * @param alignmentTypes    the alignment type
   */
  public SuperstructureState(
      Elevator.State elevator_state,
      EndEffectorWrist.State wrist_state,
      EndEffectorConstants.RollerState endEffector_state,
      Intake.State intake_state,
      Type type, AlignmentType alignmentTypes, ReefLevel level) {

    this.mElevatorState = elevator_state;
    this.mEndEffectorWristState = wrist_state;
    this.mEndEffectorRollersState = endEffector_state;
    this.mIntakeState = intake_state;
    this.mType = type;
    this.mAlignmentType = alignmentTypes;
    this.level = level;
  }

  ReefLevel level = ReefLevel.L4;
}
