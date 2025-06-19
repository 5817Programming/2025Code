package com.team5817.frc2025.autos;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.team5817.frc2025.autos.Modes.Center11;
import com.team5817.frc2025.autos.Modes.Center12;
import com.team5817.frc2025.autos.Modes.CustomMode;
import com.team5817.frc2025.autos.Modes.DoNothingMode;
import com.team5817.frc2025.subsystems.Superstructure;
import com.team5817.frc2025.subsystems.Superstructure.GoalState;
import com.team5817.frc2025.subsystems.Drive.Drive;

/**
 * This class is responsible for selecting the autonomous mode for the robot.
 */
public class AutoModeFactory {
  private Drive d;
  private Superstructure s;

  public enum ScoringLocation {
    _7A,
    _7B,
    _8A,
    _8B,
    _3A,
    _6B
  }

  public enum PickupLocation {
    FAR("FH"),
    GROUND("G", GoalState.GROUND_CORAL_INTAKE),
    CLOSE("CH"),
    PRESTAGED1("P1", GoalState.GROUND_CORAL_INTAKE),
    PRESTAGED2("P2", GoalState.GROUND_CORAL_INTAKE),
    PRESTAGED3("P3", GoalState.GROUND_CORAL_INTAKE);

    public String name = "";
    public GoalState state = GoalState.ZERO;

    PickupLocation(String name, GoalState ground) {
      this.name = name;
      this.state = ground;
    }

    PickupLocation(String name) {
      this(name, GoalState.HUMAN_CORAL_INTAKE);
    }
  }

  public enum DesiredMode {
    DO_NOTHING,
    CUSTOM_MODE,
    ONE_NET,
    TWO_NET,
    SIDE_MAIN
  }

  public enum StartingPosition {

    PROCCESSOR_SIDE("S", DesiredMode.CUSTOM_MODE, DesiredMode.SIDE_MAIN, DesiredMode.DO_NOTHING),
    CENTER_PROCESS("C", DesiredMode.CUSTOM_MODE, DesiredMode.ONE_NET, DesiredMode.TWO_NET, DesiredMode.DO_NOTHING),
    CENTER_BLANK("C", true, DesiredMode.CUSTOM_MODE, DesiredMode.DO_NOTHING),
    BLANK_SIDE("S", true, DesiredMode.CUSTOM_MODE, DesiredMode.SIDE_MAIN, DesiredMode.DO_NOTHING);

    public List<DesiredMode> modes;
    public Boolean mirrored = false;
    public String name = "";

    private StartingPosition(String name, DesiredMode... modes) {
      this.name = name;
      this.modes = List.of(modes);
      this.mirrored = false;
    }

    private StartingPosition(String name, Boolean mirrored, DesiredMode... modes) {
      this.name = name;
      this.modes = List.of(modes);
      this.mirrored = mirrored;
    }
  }

  private DesiredMode mCachedDesiredMode = DesiredMode.DO_NOTHING;
  private StartingPosition mCachedStartingPosition = StartingPosition.PROCCESSOR_SIDE;
  private PickupLocation mCachedFirstPickupLocation = PickupLocation.FAR;
  private PickupLocation mCachedSecondPickupLocation = PickupLocation.FAR;
  private PickupLocation mCachedThirdPickupLocation = PickupLocation.FAR;
  private ScoringLocation mCachedFirstScore = ScoringLocation._3A;
  private ScoringLocation mCachedSecondScore = ScoringLocation._7A;
  private ScoringLocation mCachedThirdScore = ScoringLocation._7B;
  private int mCachedScoreAmount = 3;

  private Optional<AutoBase> mAutoMode = Optional.empty();

  private static SendableChooser<DesiredMode> mModeChooser = new SendableChooser<>();
  private static SendableChooser<StartingPosition> mStartingPositionSelector = new SendableChooser<>();
  private static SendableChooser<PickupLocation> mFirstPickupLocationSelector = new SendableChooser<>();
  private static SendableChooser<PickupLocation> mSecondPickupLocationSelector = new SendableChooser<>();
  private static SendableChooser<PickupLocation> mThirdPickupLocationSelector = new SendableChooser<>();

  private static SendableChooser<ScoringLocation> mFirstScoreSelector = new SendableChooser<>();
  private static SendableChooser<ScoringLocation> mSecondScoreSelector = new SendableChooser<>();
  private static SendableChooser<ScoringLocation> mThirdScoreSelector = new SendableChooser<>();
  private static SendableChooser<Integer> mScoreAmountSelector = new SendableChooser<>();

  /**
   * Constructor for AutoModeSelector.
   * Initializes the SendableChoosers for starting position, pickup location, and
   * scoring locations.
   */
  public AutoModeFactory(Superstructure s, Drive d) {
    this.s = s;
    this.d = d;
    mStartingPositionSelector.setDefaultOption("Proccessor Side", StartingPosition.PROCCESSOR_SIDE);
    mStartingPositionSelector.addOption("Center Processor Side", StartingPosition.CENTER_PROCESS);
    mStartingPositionSelector.addOption("Center Blank Side", StartingPosition.CENTER_BLANK);
    mStartingPositionSelector.addOption("Blank Side", StartingPosition.BLANK_SIDE);

    mFirstPickupLocationSelector.setDefaultOption("Far", PickupLocation.FAR);
    mFirstPickupLocationSelector.addOption("Close", PickupLocation.CLOSE);
    mFirstPickupLocationSelector.addOption("Prestaged 1", PickupLocation.PRESTAGED1);
    mFirstPickupLocationSelector.addOption("Prestaged 2", PickupLocation.PRESTAGED2);
    mFirstPickupLocationSelector.addOption("Prestaged 3", PickupLocation.PRESTAGED3);
    mFirstPickupLocationSelector.addOption("Attempt Ground", PickupLocation.GROUND);

    mSecondPickupLocationSelector.setDefaultOption("Far", PickupLocation.FAR);
    mSecondPickupLocationSelector.addOption("Close", PickupLocation.CLOSE);
    mSecondPickupLocationSelector.addOption("Prestaged 1", PickupLocation.PRESTAGED1);
    mSecondPickupLocationSelector.addOption("Prestaged 2", PickupLocation.PRESTAGED2);
    mSecondPickupLocationSelector.addOption("Prestaged 3", PickupLocation.PRESTAGED3);
    mSecondPickupLocationSelector.addOption("Attempt Ground", PickupLocation.GROUND);

    mThirdPickupLocationSelector.setDefaultOption("Far", PickupLocation.FAR);
    mThirdPickupLocationSelector.addOption("Close", PickupLocation.CLOSE);
    mThirdPickupLocationSelector.addOption("Prestaged 1", PickupLocation.PRESTAGED1);
    mThirdPickupLocationSelector.addOption("Prestaged 2", PickupLocation.PRESTAGED2);
    mThirdPickupLocationSelector.addOption("Prestaged 3", PickupLocation.PRESTAGED3);
    mThirdPickupLocationSelector.addOption("Attempt Ground", PickupLocation.GROUND);

    mFirstScoreSelector.setDefaultOption("3A", ScoringLocation._3A);
    mFirstScoreSelector.addOption("8A", ScoringLocation._8A);
    mFirstScoreSelector.addOption("8B", ScoringLocation._8B);
    mSecondScoreSelector.setDefaultOption("7A", ScoringLocation._7A);
    mSecondScoreSelector.addOption("7B", ScoringLocation._7B);
    mSecondScoreSelector.addOption("8A", ScoringLocation._8A);
    mSecondScoreSelector.addOption("8B", ScoringLocation._8B);
    mSecondScoreSelector.addOption("3A", ScoringLocation._3A);
    mSecondScoreSelector.addOption("6B", ScoringLocation._6B);
    mThirdScoreSelector.setDefaultOption("7B", ScoringLocation._7B);
    mThirdScoreSelector.addOption("7A", ScoringLocation._7A);
    mThirdScoreSelector.addOption("8A", ScoringLocation._8A);
    mThirdScoreSelector.addOption("8B", ScoringLocation._8B);
    mThirdScoreSelector.addOption("3A", ScoringLocation._3A);
    mThirdScoreSelector.addOption("6B", ScoringLocation._6B);

    mScoreAmountSelector.setDefaultOption("3", 1);
    mScoreAmountSelector.addOption("2", 2);
    mScoreAmountSelector.addOption("1", 3);

  }

  /**
   * Updates the mode creator based on the selected starting position and desired
   * mode.
   * Updates the cached values for pickup location and scoring locations.
   */
  public void updateModeCreator() {
    if (mCachedStartingPosition != mStartingPositionSelector.getSelected()
        && mStartingPositionSelector.getSelected() != null) {
      mModeChooser = new SendableChooser<>();
      mStartingPositionSelector.getSelected().modes.forEach(m -> mModeChooser.addOption(m.name(), m));
      mCachedStartingPosition = mStartingPositionSelector.getSelected();
    }

    DesiredMode desiredMode = mModeChooser.getSelected();

    if (desiredMode == null) {
      desiredMode = DesiredMode.DO_NOTHING;
    }
    mAutoMode = getAutoModeForParams(desiredMode);

    mCachedFirstPickupLocation = mFirstPickupLocationSelector.getSelected();
    mCachedSecondPickupLocation = mSecondPickupLocationSelector.getSelected();
    mCachedThirdPickupLocation = mThirdPickupLocationSelector.getSelected();
    mCachedFirstScore = mFirstScoreSelector.getSelected();
    mCachedSecondScore = mSecondScoreSelector.getSelected();
    mCachedThirdScore = mThirdScoreSelector.getSelected();
    mCachedScoreAmount = mScoreAmountSelector.getSelected();

    SmartDashboard.putData("Starting Position", mStartingPositionSelector);
    SmartDashboard.putData("Auto Mode", mModeChooser);
    Logger.recordOutput("Selected Auto", desiredMode);

    if (desiredMode == DesiredMode.CUSTOM_MODE) {
      SmartDashboard.putData("First Score Selector", mFirstScoreSelector);
      SmartDashboard.putData("Second Score Selector", mSecondScoreSelector);
      SmartDashboard.putData("Third Score Selector", mThirdScoreSelector);
      SmartDashboard.putData("Score Amount", mScoreAmountSelector);

      SmartDashboard.putData("First Pickup Selector", mFirstPickupLocationSelector);
      SmartDashboard.putData("Second Pickup Selector", mSecondPickupLocationSelector);
      SmartDashboard.putData("Third Pickup Selector", mThirdPickupLocationSelector);
    }

  }

  /**
   * Returns the AutoBase instance for theputp given desired mode.
   *
   * @param mode The desired autonomous mode.
   * @return An Optional containing the AutoBase instance if a valid mode is
   *         found, otherwise an empty Optional.
   */
  private Optional<AutoBase> getAutoModeForParams(DesiredMode mode) {
    switch (mode) {
      case DO_NOTHING:
        return Optional.of(new DoNothingMode());
      case CUSTOM_MODE:
        return Optional.of(new CustomMode(mCachedStartingPosition, mCachedFirstPickupLocation,
            mCachedSecondPickupLocation, mCachedThirdPickupLocation, mCachedFirstScore, mCachedSecondScore,
            mCachedThirdScore, mCachedScoreAmount, s, d));
      case ONE_NET:
        return Optional.of(new Center11(mCachedStartingPosition, s, d));
      case TWO_NET:
        return Optional.of(new Center12(mCachedStartingPosition, s, d));
      case SIDE_MAIN:
        return Optional.of(new CustomMode(mCachedStartingPosition, PickupLocation.GROUND, PickupLocation.GROUND,
            PickupLocation.FAR, ScoringLocation._8A, ScoringLocation._7A, ScoringLocation._7B, 3, s, d));
      default:
        System.out.println("ERROR: unexpected auto mode: " + mode);
        break;
    }

    System.err.println("No valid auto mode found for  " + mode);
    return Optional.empty();
  }

  /**
   * Returns the SendableChooser for selecting the desired mode.
   *
   * @return The SendableChooser for desired mode.
   */
  public static SendableChooser<DesiredMode> getModeChooser() {
    return mModeChooser;
  }

  /**
   * Returns the cached desired autonomous mode.
   *
   * @return The cached desired mode.
   */
  public DesiredMode getDesiredAutomode() {
    return mCachedDesiredMode;
  }

  /**
   * Resets the AutoModeSelector by clearing the cached auto mode and desired
   * mode.
   */
  public void reset() {
    mAutoMode = Optional.empty();
    mCachedDesiredMode = null;
  }

  /**
   * Outputs the selected autonomous mode and starting position to the
   * SmartDashboard.
   */
  public void outputToSmartDashboard() {
    SmartDashboard.putString("AutoModeSelected", mCachedDesiredMode.name());
    SmartDashboard.putString("Starting Position Selected", mCachedStartingPosition.name());
  }

  /**
   * Returns the currently selected AutoBase instance.
   *
   * @return An Optional containing the AutoBase instance if present, otherwise an
   *         empty Optional.
   */
  public Optional<AutoBase> getAutoMode() {

    if (!mAutoMode.isPresent()) {
      return Optional.empty();
    }

    return mAutoMode;
  }
}
