package com.team5817.frc2025.subsystems.Drive;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.RobotConfig;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.motion.MotionProfileConstraints;
import com.team254.lib.swerve.SwerveDriveKinematics;
import com.team254.lib.util.SynchronousPIDF;
import com.team5817.frc2025.RobotConstants;
import com.team5817.frc2025.generated.TunerConstants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/**
 * Constants related to the Swerve drive system.
 */
public final class SwerveConstants {

  public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

  /* Drivetrain Constants */
  public static final double trackWidth = Units.inchesToMeters(22.25);
  public static final double wheelBase = Units.inchesToMeters(22.25);

  public static final double wheelDiameter = Units.inchesToMeters(4.00);
  public static final double wheelCircumference = wheelDiameter * Math.PI;

  public static final double driveGearRatio = RobotConstants.isComp ? 6.39 : 6.25;
  public static final double angleGearRatio = RobotConstants.isComp ? 12.1 : 15.43;

  public static final Translation2d[] swerveModuleLocations = {
      new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
      new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
      new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
      new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0),
  };
  public static final edu.wpi.first.math.geometry.Translation2d[] swerveModuleLocationsWpi = {
      swerveModuleLocations[2].wpi(),
      swerveModuleLocations[0].wpi(),
      swerveModuleLocations[3].wpi(),
      swerveModuleLocations[1].wpi()
  };

  public static RobotConfig mRobotConfig;

  static {
    try {
      mRobotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }

  }

  public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(swerveModuleLocations);

  public static final edu.wpi.first.math.geometry.Translation2d[] wpiModuleLocations = {
      new edu.wpi.first.math.geometry.Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
      new edu.wpi.first.math.geometry.Translation2d(wheelBase / 2.0, trackWidth / 2.0),
      new edu.wpi.first.math.geometry.Translation2d(-wheelBase / 2.0, -trackWidth / 2.0),
      new edu.wpi.first.math.geometry.Translation2d(wheelBase / 2.0, -trackWidth / 2.0)
  };

  public static final edu.wpi.first.math.kinematics.SwerveDriveKinematics kWpiKinematics = new edu.wpi.first.math.kinematics.SwerveDriveKinematics(
      wpiModuleLocations);

  /* Swerve Profiling Values */
  public static final double maxSpeed = 5; // meters per second
  public static final double maxAcceleration = 6; // meters per second
  public static final double maxAngularVelocity = 30;
  public static final double maxAngularAcceleration = maxAcceleration /
      Math.hypot(wheelBase / 2.0, trackWidth / 2.0);

  public static final double kV = 12 * Math.PI * wheelDiameter / (driveGearRatio * maxSpeed);
  public static final double maxAutoSpeed = maxSpeed * 0.85; // Max out at 85% to ensure attainable speeds

  /* Motor Inverts */
  public static final boolean driveMotorInvert = false;
  public static final boolean angleMotorInvert = false;

  /* Angle Encoder Invert */
  public static final SensorDirectionValue canCoderInvert = SensorDirectionValue.CounterClockwise_Positive;

  /* Controller Invert */
  public static final boolean invertYAxis = false;
  public static final boolean invertRAxis = false;
  public static final boolean invertXAxis = false;

  /* Heading Controller */

  public static final SynchronousPIDF stabilizePID = new SynchronousPIDF(
      8.0,
      0.0,
      0.3);
  public static final SynchronousPIDF snapPID = new SynchronousPIDF(
      3.0,
      0.0,
      0.8,
      1.0);

  public static final SynchronousPIDF simStabilizePID = new SynchronousPIDF(
      16.0,
      0.0,
      0.6);
  public static final SynchronousPIDF simSnapPID = new SynchronousPIDF(
      6.0,
      0.0,
      0.8,
      2.0);

  public static final double kTrajectoryDeadband = .03;

  final static SwerveModuleSimulationConfig moduleConfig = new SwerveModuleSimulationConfig(
                          DCMotor.getKrakenX60(1),
                          DCMotor.getFalcon500(1),
                          TunerConstants.FrontLeft.DriveMotorGearRatio,
                          TunerConstants.FrontLeft.SteerMotorGearRatio,
                          Volts.of(TunerConstants.FrontLeft.DriveFrictionVoltage),
                          Volts.of(TunerConstants.FrontLeft.SteerFrictionVoltage),
                          Inches.of(2),
                          KilogramSquareMeters.of(TunerConstants.FrontLeft.SteerInertia),
                          1.2);
                  public final static DriveTrainSimulationConfig driveConfig = DriveTrainSimulationConfig.Default()
                          .withGyro(COTS.ofPigeon2())
                          .withSwerveModule(() -> new SwerveModuleSimulation(moduleConfig))
                        .withBumperSize(Meters.of(.89),Meters.of(.89))
                        .withRobotMass(Pounds.of(115));

  public static final double kAutoAlignAllowableDistance = 2.0; // Meters

  public static final MotionProfileConstraints kPositionMotionProfileConstraints = new MotionProfileConstraints(
      6,
      -6,
      1);

  public static final MotionProfileConstraints kHeadingMotionProfileConstraints = new MotionProfileConstraints(
      5,
      -5,
      maxAngularAcceleration * 0.5);

  public static TalonFXConfiguration DriveFXConfig(boolean inverse) {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0.kP = 0.030 * 12.0;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.000001 * 12.0;
    config.Slot0.kS = 0.1;
    config.Slot0.kV = 12 * Math.PI * wheelDiameter / (driveGearRatio * maxSpeed);

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 110;

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 90;

    config.Voltage.PeakForwardVoltage = 12.0;
    config.Voltage.PeakReverseVoltage = -12.0;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = inverse ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;

    config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0;
    config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0;
    return config;
  }

  public static TalonFXConfiguration AzimuthFXConfig(boolean inverse) {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0.kP = .7;
    config.Slot0.kI = 2;
    config.Slot0.kD = 0.0004;
    config.Slot0.kS = 0.0;
    config.Slot0.kV = 0.0;

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 80;

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 60;

    config.Voltage.PeakForwardVoltage = 12.0;
    config.Voltage.PeakReverseVoltage = -12.0;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = inverse ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;

    return config;
  }

  public static CANcoderConfiguration swerveCancoderConfig() {
    CANcoderConfiguration CANCoderConfig = new com.ctre.phoenix6.configs.CANcoderConfiguration();
    CANCoderConfig.MagnetSensor.SensorDirection = canCoderInvert;
    return CANCoderConfig;
  }

  public static final double kCancoderBootAllowanceSeconds = 10.0;
}
