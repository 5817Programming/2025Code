package com.team5817.frc2025;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.RobotConfig;
import com.team5817.frc2025.subsystems.vision.VisionDeviceConstants;
import com.team5817.lib.drivers.ServoMotorSubsystem.ServoMotorSubsystemConstants;
import com.team5817.lib.drivers.ServoMotorSubsystem.TalonFXConstants;
import com.team5817.lib.drivers.ServoMotorSubsystemWithCancoder.AbsoluteEncoderConstants;
import com.team5817.lib.swerve.SwerveModule.SwerveModuleConstants;
import com.team254.lib.drivers.CanDeviceId;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.motion.MotionProfileConstraints;
import com.team254.lib.swerve.SwerveDriveKinematics;
import com.team254.lib.swerve.SwerveKinematicLimits;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;

/**
 * Constants class holds all the robot-wide numerical or boolean constants.
 * This class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do not put anything functional in this class.
 */
public class Constants {

	/**
	 * Enum representing the different modes the robot can operate in.
	 */
	public enum Mode {
		SIM,
		REPLAY,
		REAL
	}

	public static Mode mode = Mode.SIM;//Sim or Replay, Real is auto set for real robot
	

	// Disables extra smart dashboard outputs that slow down the robot
	public static final boolean disableExtraTelemetry = false;

	// robot loop time
	public static final double kLooperDt = 0.02;

	/* Control Board */
	public static final double kJoystickThreshold = 0.2;
	public static final int kButtonGamepadPort = 1;

	public static final double stickDeadband = 0.1;

	public static final double specializedVisionTimeout = 5;

	// Timeout constants
	public static final double kLongCANTimeoutS = 0.1;
	public static final double kCANTimeoutS = .01;
	public static final Matrix<N2, N1> kStateStdDevs = VecBuilder.fill(Math.pow(0.04, 1), Math.pow(0.04, 1));
	public static final Matrix<N2, N1> kLocalMeasurementStdDevs = VecBuilder.fill(Math.pow(0.001, 1), Math.pow(0.001, 1));
	public static final double[][] fundamentalMatrix = {
			{ 0.0, 0.0, 0.0 },
			{ 0.0, 0.0, 0.0 },
			{ 0.0, 0.0, 0.0 }
	};

	public static final double kBumberSideLength = Units.inchesToMeters(36.125-3);


    public static final double kDefaultDistanceToReef = 3;
	public static final class AutoConstants{
		public static final double enterDistance = 3.2;
		public static final double exitDistance = 1.5;
		public static final double coralSpit = .1;
		public static final double intakeWait = 0;
		public static final double alignWait = 0.2;
		public static final double intakeTimeout = 0;
		public static final double scoreTimeout = .2;
	}
	/**
	 * Constants related to the Swerve drive system.
	 */
	public static final class SwerveConstants {

		public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

		/* Drivetrain Constants */
		public static final double trackWidth = Units.inchesToMeters(22.25);
		public static final double wheelBase = Units.inchesToMeters(22.25);

		public static final double wheelDiameter = Units.inchesToMeters(4.00);
		public static final double wheelCircumference = wheelDiameter * Math.PI;

		public static final double driveGearRatio = isComp? 6.39 : 6.25;
		public static final double angleGearRatio = isComp? 12.1 : 15.43;

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

		// Stabilize Heading PID Values
		public static final double kStabilizeSwerveHeadingKp = 8.0;
		public static final double kStabilizeSwerveHeadingKi = 0.0;
		public static final double kStabilizeSwerveHeadingKd = 0.3;
		public static final double kStabilizeSwerveHeadingKf = 2.0;

		// Snap Heading PID Values
		public static final double kSnapSwerveHeadingKp = 3.0;
		public static final double kSnapSwerveHeadingKi = 0.0;
		public static final double kSnapSwerveHeadingKd = 0.8;
		public static final double kSnapSwerveHeadingKf = 1.0;

		public static final double kTrajectoryDeadband = .03;

		public static final SwerveKinematicLimits kSwerveKinematicLimits = new SwerveKinematicLimits();

		static {
			kSwerveKinematicLimits.kMaxDriveVelocity = maxSpeed;
			kSwerveKinematicLimits.kMaxDriveAcceleration = 200;
			kSwerveKinematicLimits.kMaxSteeringVelocity = maxAngularVelocity;
		}
		public static final SwerveKinematicLimits kExtendedKinematicLimits = new SwerveKinematicLimits();

		static {
			kExtendedKinematicLimits.kMaxDriveVelocity = 3;
			kExtendedKinematicLimits.kMaxDriveAcceleration = 5;
			kExtendedKinematicLimits.kMaxSteeringVelocity = maxAngularVelocity;
		}

		public static final SwerveKinematicLimits kSwerveUncappedKinematicLimits = new SwerveKinematicLimits();

		static {
			kSwerveUncappedKinematicLimits.kMaxDriveVelocity = maxSpeed;
			kSwerveUncappedKinematicLimits.kMaxDriveAcceleration = Double.MAX_VALUE;
			kSwerveUncappedKinematicLimits.kMaxSteeringVelocity = maxAngularVelocity;
		}

		public static final double kAutoAlignAllowableDistance = 2.0; // Meters

		public static final MotionProfileConstraints kPositionMotionProfileConstraints = new MotionProfileConstraints(
				6,
				-6,
				1);

		public static final MotionProfileConstraints kHeadingMotionProfileConstraints = new MotionProfileConstraints(
				5,
				-5,
				maxAngularAcceleration * 0.5);

		/*** MODULE SPECIFIC CONSTANTS ***/
		/* Front Left Module - Module 0 */
		/**
		 * Constants specific to the front left swerve module.
		 */
		public static final class Mod0 {
			public static final double compAngleOffset = 180;

			public static SwerveModuleConstants SwerveModuleConstants() {
				return new SwerveModuleConstants(
						Ports.FL_DRIVE.getDeviceNumber(),
						Ports.FL_ROTATION.getDeviceNumber(),
						compAngleOffset);
			}
		}

		/* Front Right Module - Module 1 */
		/**
		 * Constants specific to the front right swerve module.
		 */
		public static final class Mod1 {
			public static final double compAngleOffset = 0;

			public static SwerveModuleConstants SwerveModuleConstants() {
				return new SwerveModuleConstants(
						Ports.FR_DRIVE.getDeviceNumber(),
						Ports.FR_ROTATION.getDeviceNumber(),
						compAngleOffset);
			}
		}

		/* Back Left Module - Module 2 */
		/**
		 * Constants specific to the back left swerve module.
		 */
		public static final class Mod2 {
			public static final double compAngleOffset = 180;

			public static SwerveModuleConstants SwerveModuleConstants() {
				return new SwerveModuleConstants(
						Ports.BL_DRIVE.getDeviceNumber(),
						Ports.BL_ROTATION.getDeviceNumber(),
						compAngleOffset);
			}
		}

		/* Back Right Module - Module 3 */
		/**
		 * Constants specific to the back right swerve module.
		 */
		public static final class Mod3 {
			public static final double compAngleOffset = 0;

			public static SwerveModuleConstants SwerveModuleConstants() {
				return new SwerveModuleConstants(
						Ports.BR_DRIVE.getDeviceNumber(),
						Ports.BR_ROTATION.getDeviceNumber(),
						compAngleOffset);
			}
		}

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
			CANCoderConfig.MagnetSensor.SensorDirection = Constants.SwerveConstants.canCoderInvert;
			return CANCoderConfig;
		}

		public static final double kCancoderBootAllowanceSeconds = 10.0;
	}

	/**
	 * Constants related to the Pose Estimator.
	 */
	public static final class PoseEstimatorConstants {
		public record CameraConfig(Pose3d offset, String config) {
		};

		public static final Matrix<N2, N1> kStateStdDevs = VecBuilder.fill(Math.pow(0.2, 1), Math.pow(0.2, 1));
		public static final Matrix<N2, N1> kLocalMeasurementStdDevs = VecBuilder.fill(Math.pow(0.01, 1),
				Math.pow(0.01, 1));

		public static VisionDeviceConstants kDomVisionDevice = new VisionDeviceConstants(); // dot 13
		public static VisionDeviceConstants kSubVisionDevice = new VisionDeviceConstants(); // dot 12

		public static List<Integer> redTagIDFilters;
		public static List<Integer> blueTagIDFilters;

		static {
			redTagIDFilters = List.of(6,7,8,9,10,11);
			blueTagIDFilters = List.of(17,18,19,20,21,22);

		
			kDomVisionDevice.kTableName = "limelight-Dom";
			kDomVisionDevice.kRobotToCamera = new Transform3d(Units.inchesToMeters(3.071), Units.inchesToMeters(7.325),
					Units.inchesToMeters(0),
					new Rotation3d(0, 0, 0));

			kSubVisionDevice.kTableName = "limelight-Sub";
			kSubVisionDevice.kRobotToCamera = new Transform3d(Units.inchesToMeters(3.071), Units.inchesToMeters(-7.325),
					Units.inchesToMeters(0),
					new Rotation3d(0, 0, 0));// TODO set these to correct values

		}

	}

	/**
	 * Constants related to the Intake Deploy subsystem.
	 */
	public static final class IntakeDeployConstants {
		// 115.93
		// 7.92
		public static final ServoMotorSubsystemConstants kDeployServoConstants = new ServoMotorSubsystemConstants();

		public static final AbsoluteEncoderConstants kDeployEncoderConstants = new AbsoluteEncoderConstants();

		static {

			kDeployServoConstants.kName = "Deploy";
			kDeployServoConstants.simIO = isComp? false:true;

			kDeployServoConstants.kMainConstants.id = Ports.INTAKE_PIVOT;
			kDeployServoConstants.kMainConstants.counterClockwisePositive = false;
			

			kDeployServoConstants.kHomePosition = 0; // degrees
			kDeployServoConstants.kRotationsPerUnitDistance = (1.0 / 360.0)*20;

			kDeployServoConstants.kMaxUnitsLimit = 100000;
			kDeployServoConstants.kMinUnitsLimit = -100000;

			kDeployServoConstants.kKp = 3.8125	;
			kDeployServoConstants.kKi = 0.0;   
			kDeployServoConstants.kKd = 0;
			kDeployServoConstants.kKa = 0;
			kDeployServoConstants.kKs = 0;
			kDeployServoConstants.kKv = .5;
			kDeployServoConstants.kKg = 0.265625;

 
			kDeployServoConstants.kCruiseVelocity = 32*360*1/20; 
			kDeployServoConstants.kAcceleration = 32*360*1/20; 

			kDeployServoConstants.kMaxForwardOutput = 12.0;
			kDeployServoConstants.kMaxReverseOutput = -12.0;

			kDeployServoConstants.kEnableSupplyCurrentLimit = true;
			kDeployServoConstants.kSupplyCurrentLimit = 80; // amps
			kDeployServoConstants.kSupplyCurrentThreshold = 80; // amps
			kDeployServoConstants.kSupplyCurrentTimeout = 0.01; // seconds

			kDeployServoConstants.kEnableStatorCurrentLimit = true;
			kDeployServoConstants.kStatorCurrentLimit = 80; // amps

			kDeployServoConstants.kNeutralMode = NeutralModeValue.Brake;

			kDeployEncoderConstants.rotor_to_sensor_ratio = 20;
			kDeployEncoderConstants.remote_encoder_port = Ports.INTAKE_CANCODER;

			kDeployServoConstants.kHomingOutput = -.3;
			kDeployServoConstants.kHomingTimeout = 0.2;
			kDeployServoConstants.kHomingVelocityWindow = 5;
		}

	}

	/**
	 * Constants related to the Intake Roller subsystem.
	 */
	public static final class IntakeRollerConstants {
		public static TalonFXConfiguration RollerFXConfig() {
			TalonFXConfiguration config = new TalonFXConfiguration();

			config.CurrentLimits.SupplyCurrentLimitEnable = true;
			config.CurrentLimits.SupplyCurrentLimit = 40.0;

			config.CurrentLimits.StatorCurrentLimitEnable = true;
			config.CurrentLimits.StatorCurrentLimit = 80.0;

			config.Voltage.PeakForwardVoltage = 12.0;
			config.Voltage.PeakReverseVoltage = -12.0;

			config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
			return config;
		}
	}

	/**
	 * Constants related to the Elevator subsystem.
	 */
	public static final class ElevatorConstants {
		// 115.93
		// 7.92
		public static final ServoMotorSubsystemConstants kElevatorServoConstants = new ServoMotorSubsystemConstants();

		static {
			kElevatorServoConstants.kName = "Elevator";

			kElevatorServoConstants.simIO = isComp? false:true;

			kElevatorServoConstants.kMainConstants.id = Ports.ELEVATOR;
			kElevatorServoConstants.kMainConstants.counterClockwisePositive = true;

			TalonFXConstants followerConstants = new TalonFXConstants();
				followerConstants.id = Ports.ELEVATOR_2;
				followerConstants.counterClockwisePositive = false;
				followerConstants.invert_sensor_phase = false;

			kElevatorServoConstants.kFollowerConstants = new TalonFXConstants[]{followerConstants};

			kElevatorServoConstants.kHomePosition = 0; // degrees
			kElevatorServoConstants.kRotationsPerUnitDistance = 72.82/1.4*3/4;

			kElevatorServoConstants.kMaxUnitsLimit = 2.035;
			kElevatorServoConstants.kMinUnitsLimit = 0.0;

			kElevatorServoConstants.kKp = 15;
			kElevatorServoConstants.kKi = 0.0;
			kElevatorServoConstants.kKd = 0.2;
			kElevatorServoConstants.kKa = 0.0;
			kElevatorServoConstants.kKs = 0.0;
			kElevatorServoConstants.kKv = .1;
			kElevatorServoConstants.kKg = 7;

			kElevatorServoConstants.kCruiseVelocity = 9999.0/kElevatorServoConstants.kRotationsPerUnitDistance; // degrees / s
			kElevatorServoConstants.kAcceleration = 300/kElevatorServoConstants.kRotationsPerUnitDistance; // degrees / s^2

			kElevatorServoConstants.kMaxForwardOutput = 12.0;
			kElevatorServoConstants.kMaxReverseOutput = -12.0;

			kElevatorServoConstants.kEnableSupplyCurrentLimit = true;
			kElevatorServoConstants.kSupplyCurrentLimit = 40; // amps
			kElevatorServoConstants.kSupplyCurrentThreshold = 40; // amps
			kElevatorServoConstants.kSupplyCurrentTimeout = 0.01; // seconds

			kElevatorServoConstants.kEnableStatorCurrentLimit = true;
			kElevatorServoConstants.kStatorCurrentLimit = 80; // amps

			kElevatorServoConstants.kNeutralMode = NeutralModeValue.Brake;
			
			kElevatorServoConstants.kHomingTimeout = 0.1;
			kElevatorServoConstants.kHomingOutput = -.15;
			kElevatorServoConstants.kHomingVelocityWindow = 0.01;

		}

		public static double kHomingZone = 0.1; // degrees
		public static final double kCoralClearHeight = 0.15; // rotations
        public static final InterpolatingDoubleTreeMap kMidOffsetMap = new InterpolatingDoubleTreeMap();
		static {
			kMidOffsetMap.put(-.112, -0.119804);
			kMidOffsetMap.put(0.0, 0.0);
		}public static final InterpolatingDoubleTreeMap kHighOffsetMap = new InterpolatingDoubleTreeMap();
		static {
			kHighOffsetMap.put(-.11, -0.0);
			kHighOffsetMap.put(0.0, 0.0);
		}

	}

	/**
	 * Constants related to the Climb subsystem.
	 */
	public static final class ClimbConstants {
		// 115.93
		// 7.92
		public static final ServoMotorSubsystemConstants kClimbServoConstants = new ServoMotorSubsystemConstants();

		static {
			kClimbServoConstants.kName = "Climb";

			kClimbServoConstants.simIO = true;

			kClimbServoConstants.kMainConstants.id = Ports.CLIMB;
			kClimbServoConstants.kMainConstants.counterClockwisePositive = false;

			kClimbServoConstants.kHomePosition = 0; // degrees
			kClimbServoConstants.kRotationsPerUnitDistance = (1.0 / 360.0) * (45.0 / 1.0);

			kClimbServoConstants.kMaxUnitsLimit = 128.1;
			kClimbServoConstants.kMinUnitsLimit = 0.0;

			kClimbServoConstants.kKp = 3.0;
			kClimbServoConstants.kKi = 0.0;
			kClimbServoConstants.kKd = 0.0;
			kClimbServoConstants.kKa = 0.0;
			kClimbServoConstants.kKs = 0.2;
			kClimbServoConstants.kKg = 0.2;

			kClimbServoConstants.kCruiseVelocity = 400.0; // degrees / s
			kClimbServoConstants.kAcceleration = 10000.0; // degrees / s^2

			kClimbServoConstants.kMaxForwardOutput = 12.0;
			kClimbServoConstants.kMaxReverseOutput = -12.0;

			kClimbServoConstants.kEnableSupplyCurrentLimit = true;
			kClimbServoConstants.kSupplyCurrentLimit = 40; // amps
			kClimbServoConstants.kSupplyCurrentThreshold = 40; // amps
			kClimbServoConstants.kSupplyCurrentTimeout = 0.01; // seconds

			kClimbServoConstants.kEnableStatorCurrentLimit = true;
			kClimbServoConstants.kStatorCurrentLimit = 80; // amps

			kClimbServoConstants.kNeutralMode = NeutralModeValue.Brake;
		}

	}

	/**
	 * Constants related to the End Effector Wrist subsystem.
	 */
	public static final class EndEffectorWristConstants {
		// 115.93
		// 7.92
		public static final ServoMotorSubsystemConstants kWristServoConstants = new ServoMotorSubsystemConstants();

		public static final AbsoluteEncoderConstants kWristEncoderConstants = new AbsoluteEncoderConstants();

		static {
			kWristServoConstants.kName = "EndEffectorWrist";

			kWristServoConstants.simIO = isComp? false:true;

			kWristServoConstants.kMainConstants.id = Ports.ENDEFFECTOR_WRIST;
			kWristServoConstants.kMainConstants.counterClockwisePositive = false;

			kWristServoConstants.kHomePosition = -89; // degrees
			kWristServoConstants.kRotationsPerUnitDistance = (1/360.0) * 10;

			kWristServoConstants.kMaxUnitsLimit = 200-89;
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

			kWristServoConstants.kHomingTimeout = .1;
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

	/**
	 * Constants related to the End Effector Roller subsystem.
	 */
	public static final class EndEffectorRollerConstants {
		public static TalonFXConfiguration RollerFXConfig() {
			TalonFXConfiguration config = new TalonFXConfiguration();

			config.CurrentLimits.SupplyCurrentLimitEnable = true;
			config.CurrentLimits.SupplyCurrentLimit = 30.0;

			config.CurrentLimits.StatorCurrentLimitEnable = true;
			config.CurrentLimits.StatorCurrentLimit = 100.0;

			config.Voltage.PeakForwardVoltage = 12.0;
			config.Voltage.PeakReverseVoltage = -12.0;

			config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
			return config;
		}
	}
	public static final boolean isComp = isComp();
	private static boolean isComp(){
		final Path commentPath = Path.of("/etc/machine-info");
		try {
			var comment = Files.readString(commentPath);
			return comment.contains("Comp");
		} catch (IOException e) {
			System.out.println(e);
			return false;
		}
		}
}
