package com.team5817.frc2025.subsystems.Intake;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team5817.frc2025.Ports;
import com.team5817.frc2025.subsystems.Rollers.IRollerState;
import com.team5817.frc2025.subsystems.Rollers.RollerConstantsTalonFX;
import com.team5817.frc2025.subsystems.Rollers.RollerSubsystem.RollerControlMode;
import com.team5817.lib.drivers.ServoConstants;

import lombok.Getter;

public class IntakeConstants {
  /**
   * Constants related to the Intake Deploy subsystem.
   */
  public static final class DeployConstants {
    public static final ServoConstants kDeployServoConstants = new ServoConstants();

    static {

      kDeployServoConstants.kName = "Deploy";

      kDeployServoConstants.kMainConstants.id = Ports.INTAKE_PIVOT;
      kDeployServoConstants.kMainConstants.counterClockwisePositive = false;

      kDeployServoConstants.kHomePosition = 0; // degrees
      kDeployServoConstants.kRotationsPerUnitDistance = (1.0 / 360.0) * 20;

      kDeployServoConstants.kMaxUnitsLimit = 100000;
      kDeployServoConstants.kMinUnitsLimit = -100000;

      kDeployServoConstants.kKp = 3.8125;
      kDeployServoConstants.kKi = 0.0;
      kDeployServoConstants.kKd = 0;
      kDeployServoConstants.kKa = 0;
      kDeployServoConstants.kKs = 0;
      kDeployServoConstants.kKv = .5;
      kDeployServoConstants.kKg = 0.265625;

      kDeployServoConstants.kCruiseVelocity = 32 * 360 * 1 / 20;
      kDeployServoConstants.kAcceleration = 32 * 360 * 1 / 20;

      kDeployServoConstants.kMaxForwardOutput = 12.0;
      kDeployServoConstants.kMaxReverseOutput = -12.0;

      kDeployServoConstants.kEnableSupplyCurrentLimit = true;
      kDeployServoConstants.kSupplyCurrentLimit = 80; // amps
      kDeployServoConstants.kSupplyCurrentThreshold = 80; // amps
      kDeployServoConstants.kSupplyCurrentTimeout = 0.01; // seconds

      kDeployServoConstants.kEnableStatorCurrentLimit = true;
      kDeployServoConstants.kStatorCurrentLimit = 80; // amps

      kDeployServoConstants.kNeutralMode = NeutralModeValue.Brake;

      kDeployServoConstants.kHomingOutput = -.3;
      kDeployServoConstants.kHomingTimeout = 0.2;
      kDeployServoConstants.kHomingVelocityWindow = 5;
    }

  }

  public static final class RollerConstants {

    public static RollerConstantsTalonFX motorConstants = new RollerConstantsTalonFX();
    static {
      motorConstants.kSupplyCurrentLimit = 40;
      motorConstants.kStatorCurrentLimit = 80;
      motorConstants.kEnableSupplyCurrentLimit = true;
      motorConstants.kEnableStatorCurrentLimit = true;
      motorConstants.kMaxForwardOutput = 12.0;
      motorConstants.kMaxReverseOutput = -12.0;
    }

    public enum FeederState implements IRollerState {
      IDLE(0),
      INTAKING(10),
      EXHAUST(-6);

      @Getter
      private final double demand;
      @Getter
      private final RollerControlMode controlMode;

      FeederState(double demand) {
        this.demand = demand;
        this.controlMode = RollerControlMode.VOLTAGE;
      }
    }

    public enum SideIndexerState implements IRollerState {
      IDLE(0),
      INTAKING(8),
      EXHAUST(-6),
      SLOW_EXAUST(-2);

      @Getter
      private final double demand;
      @Getter
      private final RollerControlMode controlMode;

      SideIndexerState(double demand) {
        this.demand = demand;
        this.controlMode = RollerControlMode.VOLTAGE;
      }
    }

    public enum LowIndexerState implements IRollerState {
      IDLE(0),
      INTAKING(2.5),
      EXHAUST(-6),
      SLOW_EXAUST(-2);

      @Getter
      private final double demand;
      @Getter
      private final RollerControlMode controlMode;

      LowIndexerState(double demand) {
        this.demand = demand;
        this.controlMode = RollerControlMode.VOLTAGE;
      }
    }
  }
}
