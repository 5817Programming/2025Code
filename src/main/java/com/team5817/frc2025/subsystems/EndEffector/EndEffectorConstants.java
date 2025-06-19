package com.team5817.frc2025.subsystems.EndEffector;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team5817.frc2025.Ports;
import com.team5817.frc2025.subsystems.Rollers.IRollerState;
import com.team5817.frc2025.subsystems.Rollers.RollerConstantsTalonFX;
import com.team5817.frc2025.subsystems.Rollers.RollerSubsystem.RollerControlMode;
import com.team5817.lib.drivers.ServoConstants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import lombok.Getter;

public class EndEffectorConstants {
  public static final RollerConstantsTalonFX rollerConstants = new RollerConstantsTalonFX();
  static {
    rollerConstants.kMaxForwardOutput = 12;
    rollerConstants.kMaxReverseOutput = -12;
    rollerConstants.kNeutralMode = NeutralModeValue.Brake;
    rollerConstants.kSupplyCurrentLimit = 30;
    rollerConstants.kStatorCurrentLimit = 100;
    rollerConstants.counterClockwisePositive = false;
    rollerConstants.kEnableStatorCurrentLimit = true;
    rollerConstants.kEnableSupplyCurrentLimit = true;
  }

  public enum RollerState implements IRollerState {
    IDLE(0.0),
    HOLD(-2.0),
    HOLDCORAL(-1.5),
    CORAL_INTAKE(-4.0),
    l4(12.0),
    l3(3.0),
    l2(8.0),
    l1(1.5),
    ALGAE_INTAKE(-9.0),
    ALGAE_OUTTAKE(1.25),
    ALGAE_SHOOT(12);

    @Getter
    double demand = 0;
    @Getter
    RollerControlMode controlMode = RollerControlMode.VOLTAGE;

    RollerState(double demand) {
      this.demand = demand;
    }
  }

  /**
   * Constants related to the End Effector Wrist subsystem.
   */
  public static final class EndEffectorWristConstants {

    public static final ServoConstants kWristServoConstants = new ServoConstants();

    static {
      kWristServoConstants.kName = "EndEffectorWrist";

      kWristServoConstants.kMainConstants.id = Ports.ENDEFFECTOR_WRIST;
      kWristServoConstants.kMainConstants.counterClockwisePositive = false;

      kWristServoConstants.kHomePosition = 0; // degrees
      kWristServoConstants.kRotationsPerUnitDistance = (1 / 360.0) * 10;

      kWristServoConstants.kMaxUnitsLimit = 200 - 89;
      kWristServoConstants.kMinUnitsLimit = -89;

      kWristServoConstants.kKp = 100;
      kWristServoConstants.kKi = 0;
      kWristServoConstants.kKd = 0.1;
      kWristServoConstants.kKa = 0.0;
      kWristServoConstants.kKs = 0.04;
      kWristServoConstants.kKv = 0;
      kWristServoConstants.kKg = 3;
      kWristServoConstants.kGravityType = GravityTypeValue.Arm_Cosine;

      kWristServoConstants.kCruiseVelocity = 160000; // degrees / s
      kWristServoConstants.kAcceleration = 9000.0; // degrees / s^2

      kWristServoConstants.kMaxForwardOutput = 12.0;
      kWristServoConstants.kMaxReverseOutput = -12.0;

      kWristServoConstants.kEnableSupplyCurrentLimit = true;
      kWristServoConstants.kSupplyCurrentLimit = 40; // amps
      kWristServoConstants.kSupplyCurrentThreshold = 40; // amps
      kWristServoConstants.kSupplyCurrentTimeout = 0.01; // seconds

      kWristServoConstants.kEnableStatorCurrentLimit = true;
      kWristServoConstants.kStatorCurrentLimit = 30; // amps

      kWristServoConstants.kNeutralMode = NeutralModeValue.Brake;

      kWristServoConstants.kHomingTimeout = .5;
      kWristServoConstants.kHomingOutput = -.25;
      kWristServoConstants.kHomingVelocityWindow = 1;
    }
    public static final InterpolatingDoubleTreeMap kHighOffsetMap = new InterpolatingDoubleTreeMap();
    static {
      kHighOffsetMap.put(-.11, 27.0);
      kHighOffsetMap.put(0.0, 0.0);
    }
    public static final InterpolatingDoubleTreeMap kMidOffsetMap = new InterpolatingDoubleTreeMap();
    static {
      kMidOffsetMap.put(-.11, 0.0);
      kMidOffsetMap.put(0.0, 0.0);
    }
  }

}
