package com.team5817.frc2025.generated;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
import edu.wpi.first.units.measure.*;

public class TunerConstants {

  // The steer motor uses any SwerveModule.SteerRequestType control request with
  private static final Slot0Configs steerGains = new Slot0Configs()
      .withKP(100)
      .withKI(0)
      .withKD(0.5)
      .withKS(0.1)
      .withKV(1.91);
  // When using closed-loop control, the drive motor uses the control
  // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
  private static final Slot0Configs driveGains = new Slot0Configs()
      .withKP(0.1)
      .withKI(0)
      .withKD(0)
      .withKV(0.124);

  // The closed-loop output type to use for the steer motors;
  // This affects the PID/FF gains for the steer motors
  private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
  // The closed-loop output type to use for the drive motors;
  // This affects the PID/FF gains for the drive motors
  private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

  // The type of motor used for the drive motor
  private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;

  // The type of motor used for the steer motor
  private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

  // The remote sensor feedback type to use for the steer motors;
  // When not Pro-licensed, FusedCANcoder/SyncCANcoder automatically fall back to RemoteCANcoder
  private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.RemoteCANcoder;

  // === Current limits ===

  // The stator current at which the wheels start to slip;
  // This needs to be tuned to your individual robot
  private static final Current kSlipCurrent = Amps.of(120.0);

  // === Motor configs ===

  private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();

  private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
      .withCurrentLimits(new CurrentLimitsConfigs()
          // Swerve azimuth does not require much torque output,
          // so we can set a relatively low stator current limit
          // to help avoid brownouts without impacting performance.
          .withStatorCurrentLimit(Amps.of(60))
          .withStatorCurrentLimitEnable(true));

  // Initial configs for the azimuth encoder
  private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();

  // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
  private static final Pigeon2Configuration pigeonConfigs = null;

  // CAN bus that the swerve devices are located on;
  // All swerve modules must share the same CAN bus
  public static final CANBus kCANBus = new CANBus("canivore1");
  public static final CANBus Pigeon2Bus = new CANBus("rio");

  // === Drivetrain-wide co
  // Theoretical free speed (m/s) at 12 V applied output;
  // This needs to be tuned to your individual robot
  public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(20);

  // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
  // This may need to be tuned to your individual robot
  private static final double kCoupleRatio = 0;

  // Gear ratios (from module config)
  private static final double kDriveGearRatio = 6.394736842105262;
  private static final double kSteerGearRatio = 12.1;

  // Radius of the wheel
  private static final Distance kWheelRadius = Inches.of(2.0);

  // Inversion settings for drivetrain sides
  private static final boolean kInvertLeftSide = true;//TODO flip these back if the robot drives wrong
  private static final boolean kInvertRightSide = false;//TODO flip these back if the robot drives wrong

  // CAN ID for the Pigeon 2
  private static final int kPigeonId = 23;


  // Moment of inertia for steer and drive motors
  private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.004);
  private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.025);

  // Simulated voltage necessary to overcome friction
  private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
  private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

  // === Drivetrain config ===

  public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
      .withCANBusName(kCANBus.getName())
      .withPigeon2Id(kPigeonId)
      .withPigeon2Configs(pigeonConfigs);

  private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator =
      new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
          .withDriveMotorGearRatio(kDriveGearRatio)
          .withSteerMotorGearRatio(kSteerGearRatio)
          .withCouplingGearRatio(kCoupleRatio)
          .withWheelRadius(kWheelRadius)
          .withSteerMotorGains(steerGains)
          .withDriveMotorGains(driveGains)
          .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
          .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
          .withSlipCurrent(kSlipCurrent)
          .withSpeedAt12Volts(kSpeedAt12Volts)
          .withDriveMotorType(kDriveMotorType)
          .withSteerMotorType(kSteerMotorType)
          .withFeedbackSource(kSteerFeedbackType)
          .withDriveMotorInitialConfigs(driveInitialConfigs)
          .withSteerMotorInitialConfigs(steerInitialConfigs)
          .withEncoderInitialConfigs(encoderInitialConfigs)
          .withSteerInertia(kSteerInertia)
          .withDriveInertia(kDriveInertia)
          .withSteerFrictionVoltage(kSteerFrictionVoltage)
          .withDriveFrictionVoltage(kDriveFrictionVoltage);

  // === Module positions (based on 22.5" square wheelbase) ===

  private static final Distance kX = Inches.of(11.25);
  private static final Distance kY = Inches.of(11.25);

  // === Module constants ===
  public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft =
      ConstantCreator.createModuleConstants(
          5, 1, 1, Rotations.of(-0.250244140625+0.5),//TODO find out if the +0.5 and invert drive motor fixes the rotation direction
          kX, kY, kInvertLeftSide, false, false);

  public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight =
      ConstantCreator.createModuleConstants(
          6, 2, 2, Rotations.of(0.338623046875+0.5),//TODO find out if the +0.5 and invert drive motor fixes the rotation direction
          kX, kY.negate(), kInvertRightSide, false, false);

  public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft =
      ConstantCreator.createModuleConstants(
          7, 3, 3, Rotations.of(0-0.349609375+0.5),//TODO find out if the +0.5 and invert drive motor fixes the rotation direction
          kX.negate(), kY, kInvertLeftSide, false, false);

  public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight =
      ConstantCreator.createModuleConstants(
          8, 4, 4, Rotations.of(0.281005859375+0.5),//TODO find out if the +0.5 and invert drive motor fixes the rotation direction
          kX.negate(), kY.negate(), kInvertRightSide, false, false);
}
