package com.team5817.frc2025.subsystems.Elevator;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team5817.frc2025.Ports;
import com.team5817.lib.drivers.ServoConstants;
import com.team5817.lib.drivers.ServoMotorSubsystem.TalonFXConstants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * Constants related to the Elevator subsystem.
 */
public final class ElevatorConstants {
  // 115.93
  // 7.92
  public static final ServoConstants kElevatorServoConstants = new ServoConstants();

  static {
    kElevatorServoConstants.kName = "Elevator";

    kElevatorServoConstants.kMainConstants.id = Ports.ELEVATOR;
    kElevatorServoConstants.kMainConstants.counterClockwisePositive = true;

    TalonFXConstants followerConstants = new TalonFXConstants();
    followerConstants.id = Ports.ELEVATOR_2;
    followerConstants.counterClockwisePositive = false;
    followerConstants.invert_sensor_phase = false;

    kElevatorServoConstants.kFollowerConstants = new TalonFXConstants[] { followerConstants };

    kElevatorServoConstants.kHomePosition = 0; // degrees
    kElevatorServoConstants.kRotationsPerUnitDistance = 72.82 / 1.4 * 3 / 4;

    kElevatorServoConstants.kMaxUnitsLimit = 2.035;
    kElevatorServoConstants.kMinUnitsLimit = 0.0;

    kElevatorServoConstants.kKp = 15;
    kElevatorServoConstants.kKi = 0.0;
    kElevatorServoConstants.kKd = 0.2;
    kElevatorServoConstants.kKa = 0.0;
    kElevatorServoConstants.kKs = 0.0;
    kElevatorServoConstants.kKv = .1;
    kElevatorServoConstants.kKg = 7;

    kElevatorServoConstants.kCruiseVelocity = 9999.0 / kElevatorServoConstants.kRotationsPerUnitDistance; // degrees / s
    kElevatorServoConstants.kAcceleration = 300 / kElevatorServoConstants.kRotationsPerUnitDistance; // degrees / s^2

    kElevatorServoConstants.kMaxForwardOutput = 12.0;
    kElevatorServoConstants.kMaxReverseOutput = -12.0;

    kElevatorServoConstants.kEnableSupplyCurrentLimit = true;
    kElevatorServoConstants.kSupplyCurrentLimit = 40; // amps
    kElevatorServoConstants.kSupplyCurrentThreshold = 40; // amps
    kElevatorServoConstants.kSupplyCurrentTimeout = 0.01; // seconds

    kElevatorServoConstants.kEnableStatorCurrentLimit = true;
    kElevatorServoConstants.kStatorCurrentLimit = 80; // amps

    kElevatorServoConstants.kFollowerOpposeMasterDirection = true;

    kElevatorServoConstants.kNeutralMode = NeutralModeValue.Brake;

    kElevatorServoConstants.kHomingTimeout = 0.5;
    kElevatorServoConstants.kHomingOutput = -.25;
    kElevatorServoConstants.kHomingVelocityWindow = 0.1;

  }

  public static double kHomingZone = 0.1; // degrees
  public static final double kCoralClearHeight = 0.15; // rotations
  public static final InterpolatingDoubleTreeMap kMidOffsetMap = new InterpolatingDoubleTreeMap();
  static {
    kMidOffsetMap.put(-.112, -0.149804);
    kMidOffsetMap.put(0.0, 0.0);
  }
  public static final InterpolatingDoubleTreeMap kHighOffsetMap = new InterpolatingDoubleTreeMap();
  static {
    kHighOffsetMap.put(-.11, -0.0);
    kHighOffsetMap.put(0.0, 0.0);
  }

}
