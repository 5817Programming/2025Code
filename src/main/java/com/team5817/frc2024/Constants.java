package com.team5817.frc2024;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.team5817.frc2024.subsystems.limelight.GoalTracker;
import com.team5817.frc2024.subsystems.vision.VisionDeviceConstants;
import com.team5817.lib.swerve.SwerveModule.SwerveModuleConstants;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.swerve.SwerveDriveKinematics;
import com.team254.lib.swerve.SwerveKinematicLimits;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;

public class Constants {

	public enum Mode{
		SIM,
		REPLAY
	}
	public static Mode mode = Mode.SIM;

	// Disables extra smart dashboard outputs that slow down the robot
	public static final boolean disableExtraTelemetry = false;

	// robot loop time
	public static final double kLooperDt = 0.02;

	/* Control Board */
	public static final double kJoystickThreshold = 0.2;
	public static final int kButtonGamepadPort = 1;

	public static final double stickDeadband = 0.15;

	// Timeout constants
	public static final int kLongCANTimeoutMs = 100;
	public static final int kCANTimeoutMs = 10;

    public static final double[][] fundamentalMatrix = 
	{
	{0.0,0.0,0.0},
	{0.0,0.0,0.0},
	{0.0,0.0,0.0}};

    public static final double SwerveMaxspeedMPS = 6;//TODO FIND

	public static final class SwerveConstants {


		public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

		/* Drivetrain Constants */
		public static final double trackWidth = Units.inchesToMeters(20.75);
		public static final double wheelBase = Units.inchesToMeters(20.75);

		public static final double wheelDiameter = Units.inchesToMeters(4.00);
		public static final double wheelCircumference = wheelDiameter * Math.PI;

		public static final double driveGearRatio = 7.13; //TODO
		public static final double angleGearRatio = (150.0 / 7.0); //TODO

		public static final Translation2d[] swerveModuleLocations = {
			new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
			new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
			new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
			new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
		};

		public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(swerveModuleLocations);

		public static final edu.wpi.first.math.geometry.Translation2d[] wpiModuleLocations = {
			new edu.wpi.first.math.geometry.Translation2d(wheelBase / 2.0, trackWidth / 2.0),
			new edu.wpi.first.math.geometry.Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
			new edu.wpi.first.math.geometry.Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
			new edu.wpi.first.math.geometry.Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
		};

		public static final edu.wpi.first.math.kinematics.SwerveDriveKinematics kWpiKinematics =
				new edu.wpi.first.math.kinematics.SwerveDriveKinematics(wpiModuleLocations);

		/* Swerve Profiling Values */
		public static final double maxSpeed = 5.2; // meters per second
		public static final double maxAngularVelocity = 11.5;

		public static final double kV = 12 * Math.PI * wheelDiameter / (driveGearRatio * maxSpeed);
		public static final double maxAutoSpeed = maxSpeed * 0.85; // Max out at 85% to ensure attainable speeds

		/* Motor Inverts */
		public static final boolean driveMotorInvert = false;
		public static final boolean angleMotorInvert = true;

		/* Angle Encoder Invert */
		public static final SensorDirectionValue canCoderInvert = SensorDirectionValue.CounterClockwise_Positive;

		/* Controller Invert */
		public static final boolean invertYAxis = false;
		public static final boolean invertRAxis = false;
		public static final boolean invertXAxis = false;

		/* Heading Controller */

		// Stabilize Heading PID Values
		public static final double kStabilizeSwerveHeadingKp = 10.0;
		public static final double kStabilizeSwerveHeadingKi = 0.0;
		public static final double kStabilizeSwerveHeadingKd = 0.3;
		public static final double kStabilizeSwerveHeadingKf = 2.0;

		// Snap Heading PID Values
		public static final double kSnapSwerveHeadingKp = 10.0;
		public static final double kSnapSwerveHeadingKi = 0.0;
		public static final double kSnapSwerveHeadingKd = 0.6;
		public static final double kSnapSwerveHeadingKf = 1.0;

		public static final SwerveKinematicLimits kSwerveKinematicLimits = new SwerveKinematicLimits();

		static {
			kSwerveKinematicLimits.kMaxDriveVelocity = maxSpeed;
			kSwerveKinematicLimits.kMaxDriveAcceleration = Double.MAX_VALUE;
			kSwerveKinematicLimits.kMaxSteeringVelocity = maxAngularVelocity;
		}
		
		public static final SwerveKinematicLimits kSwerveUncappedKinematicLimits = new SwerveKinematicLimits();

		static {
			kSwerveKinematicLimits.kMaxDriveVelocity = maxSpeed;
			kSwerveKinematicLimits.kMaxDriveAcceleration = Double.MAX_VALUE;
			kSwerveKinematicLimits.kMaxSteeringVelocity = Double.MAX_VALUE;
		}
	
	

		/*** MODULE SPECIFIC CONSTANTS ***/
		/* Front Left Module - Module 0 */
		public static final class Mod0 {
			public static final double compAngleOffset = 99.5;
			public static final double epsilonAngleOffset = 0.87;

			public static SwerveModuleConstants SwerveModuleConstants() {
				return new SwerveModuleConstants(
						Ports.FL_DRIVE.getDeviceNumber(),
						Ports.FL_ROTATION.getDeviceNumber(),
						compAngleOffset);
			}
		}

		/* Front Right Module - Module 1 */
		public static final class Mod1 {
			public static final double compAngleOffset = 254.880;
			public static final double epsilonAngleOffset = 67.68;

			public static SwerveModuleConstants SwerveModuleConstants() {
				return new SwerveModuleConstants(
						Ports.FR_DRIVE.getDeviceNumber(),
						Ports.FR_ROTATION.getDeviceNumber(),
						compAngleOffset);
			}
		}

		/* Back Left Module - Module 2 */
		public static final class Mod2 {
			public static final double compAngleOffset = 180.0;
			public static final double epsilonAngleOffset = 46.8;

			public static SwerveModuleConstants SwerveModuleConstants() {
				return new SwerveModuleConstants(
						Ports.BL_DRIVE.getDeviceNumber(),
						Ports.BL_ROTATION.getDeviceNumber(),
						compAngleOffset);
			}
		}

		/* Back Right Module - Module 3 */
		public static final class Mod3 {
			public static final double compAngleOffset = 181.5822;
			public static final double epsilonAngleOffset = 61.56;

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
			config.MotorOutput.Inverted = inverse ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;


			config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;
			config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25;
			return config;
		}

		public static TalonFXConfiguration AzimuthFXConfig(boolean inverse) {
			TalonFXConfiguration config = new TalonFXConfiguration();
			

			config.Slot0.kP = 1.0005;
			config.Slot0.kI = 0.0;
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
			config.MotorOutput.Inverted = inverse ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

			return config;
		}

		public static CANcoderConfiguration swerveCancoderConfig() {
			CANcoderConfiguration CANCoderConfig = new com.ctre.phoenix6.configs.CANcoderConfiguration();
			CANCoderConfig.MagnetSensor.SensorDirection = Constants.SwerveConstants.canCoderInvert;
			return CANCoderConfig;
		}

		public static final double kCancoderBootAllowanceSeconds = 10.0;
	}

	public static final class PoseEstimatorConstants {
		public record CameraConfig(Pose3d offset, String config) {}
		;

		public static final Matrix<N2, N1> kStateStdDevs = VecBuilder.fill(Math.pow(0.2, 1), Math.pow(0.2, 1));
		public static final Matrix<N2, N1> kLocalMeasurementStdDevs =
				VecBuilder.fill(Math.pow(0.01, 1), Math.pow(0.01, 1));


	public static VisionDeviceConstants kDomVisionDevice = new VisionDeviceConstants(); // dot 13
	public static VisionDeviceConstants kSubVisionDevice = new VisionDeviceConstants(); // dot 12

	static {
		kDomVisionDevice.kTableName = "limelight-Dom";
		kDomVisionDevice.kRobotToCamera = new Transform3d(Units.inchesToMeters(3.071), Units.inchesToMeters(7.325),Units.inchesToMeters(0), 
				new Rotation3d(0,0,0));

		kSubVisionDevice.kTableName = "limelight-Sub";
		kSubVisionDevice.kRobotToCamera = new Transform3d(Units.inchesToMeters(3.071), Units.inchesToMeters(-7.325), Units.inchesToMeters(0),
				new Rotation3d(0,0,0));//TODO set these to correct values

	}

}

	public static final class LimelightConstants {

		public static final double kNoteHeight = 0.0508;
		public static final double kNoteTargetOffset = 0.2;
		public static final double kMaxNoteTrackingDistance = 6.75;
		public static final double kNoteTrackEpsilon = 1.0;

		public static final String kName = "limelight";
		public static final Translation2d kRobotToCameraTranslation = new Translation2d(0.0, 0.0);
		public static final double kCameraHeightMeters = 0.65;
		public static final Rotation2d kCameraPitch = Rotation2d.fromDegrees(-18.0);
		public static final Rotation2d kCameraYaw = Rotation2d.fromDegrees(0.0);

		public static final GoalTracker.Configuration kNoteTrackerConstants = new GoalTracker.Configuration();

		static {
			kNoteTrackerConstants.kMaxTrackerDistance = 0.46;
			kNoteTrackerConstants.kMaxGoalTrackAge = 0.5;
			kNoteTrackerConstants.kCameraFrameRate = 30.0;
			kNoteTrackerConstants.kStabilityWeight = 1.0;
			kNoteTrackerConstants.kAgeWeight = 0.2;
			kNoteTrackerConstants.kSwitchingWeight = 0.2;
		}
	}
}
