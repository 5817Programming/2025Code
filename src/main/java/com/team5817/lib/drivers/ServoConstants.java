package com.team5817.lib.drivers;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team5817.lib.drivers.ServoMotorSubsystem.TalonFXConstants;

public class ServoConstants {
  public String kName = "ERROR_ASSIGN_A_NAME";

  public double kLooperDt = 0.01;
  public double kCANTimeout = 0.010; // use for important on the fly updates
  public int kLongCANTimeoutMs = 100; // use for constructors

  public TalonFXConstants kMainConstants = new TalonFXConstants();
  public TalonFXConstants[] kFollowerConstants = new TalonFXConstants[0];

  public GravityTypeValue kGravityType = GravityTypeValue.Elevator_Static;
  public NeutralModeValue kNeutralMode = NeutralModeValue.Brake;
  public double kHomePosition = 0.0; // Units
  public double kRotationsPerUnitDistance = 1.0;
  public double kSoftLimitDeadband = 0.0;
  public double kKp = 0; // Raw output / raw error
  public double kKi = 0; // Raw output / sum of raw error
  public double kKd = 0; // Raw output / (err - prevErr)
  public double kKv = 0;
  public double kKa = 0;
  public double kKs = 0;
  public double kKg = 0;
  public int kDeadband = 0; // rotation

  public double kPositionKp = 0;
  public double kPositionKi = 0;
  public double kPositionKd = 0;
  public double kPositionKf = 0;
  public int kPositionDeadband = 0; // Ticks

  public double kVelocityFeedforward = 0;
  public double kArbitraryFeedforward = 0;
  public double kCruiseVelocity = 0; // Units/s
  public double kAcceleration = 0; // Units/s^2
  public double kJerk = 0; // Units/s^3
  public double kRampRate = 0.0; // s

  public int kSupplyCurrentLimit = 60; // amps
  public int kSupplyCurrentThreshold = 60;// TODO look at what this should be add to roller if good
  public double kSupplyCurrentTimeout = 0.0; // Seconds TODO look at what this should be add to roller if good
  public boolean kEnableSupplyCurrentLimit = false;

  public int kStatorCurrentLimit = 40; // amps
  public boolean kEnableStatorCurrentLimit = false;

  public double kMaxForwardOutput = 12.0; // Volts
  public double kMaxReverseOutput = -12.0; // Voltsa

  public boolean kFollowerOpposeMasterDirection = false;

  public double kMaxUnitsLimit = Double.POSITIVE_INFINITY;
  public double kMinUnitsLimit = Double.NEGATIVE_INFINITY;

  public int kStatusFrame8UpdateRate = 1000;

  public double kHomingTimeout = 0;
  public double kHomingVelocityWindow = 0;
  public double kHomingOutput = 0;

  /**
   * Converts rotations to units.
   *
   * @param rotations The rotations.
   * @return The units.
   */
  protected double rotationsToUnits(double rotations) {
    return rotations / kRotationsPerUnitDistance;
  }

  /**
   * Converts rotations to homed units.
   *
   * @param rotations The rotations.
   * @return The homed units.
   */
  protected double rotationsToHomedUnits(double rotations) {
    double val = rotationsToUnits(rotations);
    return val + kHomePosition;
  }

  /**
   * Converts units to rotations.
   *
   * @param units The units.
   * @return The rotations.
   */
  protected double unitsToRotations(double units) {
    return units * kRotationsPerUnitDistance;
  }

  /**
   * Converts home-aware units to rotations.
   *
   * @param units The units.
   * @return The rotations.
   */
  protected double homeAwareUnitsToRotations(double units) {
    return unitsToRotations(units - kHomePosition);
  }

  public final double mForwardSoftLimitRotations = (((kMaxUnitsLimit - kHomePosition)
      * kRotationsPerUnitDistance)
      - kSoftLimitDeadband);
  public final double mReverseSoftLimitRotations = (((kMinUnitsLimit - kHomePosition)
      * kRotationsPerUnitDistance)
      + kSoftLimitDeadband);
}
