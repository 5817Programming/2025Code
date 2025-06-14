package com.team5817.lib.swerve;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team5817.frc2025.RobotConstants;
import com.team5817.frc2025.subsystems.Drive.SwerveConstants;
import com.team5817.lib.Conversions;
import com.team5817.lib.RobotMode;
import com.team5817.lib.Util;
import com.team5817.lib.drivers.Subsystem;
import com.team254.lib.drivers.Phoenix6Util;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.swerve.SwerveModuleState;

import edu.wpi.first.wpilibj.Timer;

public class SwerveModule extends Subsystem {

  private int kModuleNumber;
  private double kAngleOffset;

  private TalonFX mAngleMotor;
  private TalonFX mDriveMotor;
  private CANcoder angleEncoder;

  private BaseStatusSignal[] mSignals = new BaseStatusSignal[4];

  private ModuleInputsAutoLogged mInputs = new ModuleInputsAutoLogged();
  private ModuleOutputs mOutputs = new ModuleOutputs();

  public enum DriveType {
    OPENLOOP,
    VELOCITY,
    POSITION,
    NUETRAL
  }

  @AutoLog
  public static class ModuleInputs {
    // Inputs
    public double timestamp = 0.0;
    public double rotationPosition = 0.0;
    public double drivePosition = 0.0;
    public double driveVelocity = 0.0;
    public double absolutePosition = 0.0;
  }

  public static class ModuleOutputs {
    // Outputs
    public double driveDemand = 0.0;
    public double driveVelocity = 0.0;
    public double rotTarget = 0.0;
    public DriveType driveType = DriveType.OPENLOOP;
    public DriveType rotType = DriveType.POSITION;

  }

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants, CANcoder cancoder) {
    this.kModuleNumber = moduleNumber;
    kAngleOffset = moduleConstants.angleOffset;

    angleEncoder = cancoder;

    // Angle motor config
    mAngleMotor = new TalonFX(moduleConstants.angleMotorID, "canivore1");
    Phoenix6Util.checkErrorAndRetry(() -> mAngleMotor.getConfigurator()
        .apply(SwerveConstants.AzimuthFXConfig(SwerveConstants.angleMotorInvert), RobotConstants.kLongCANTimeoutS));

    // Drive motor config
    mDriveMotor = new TalonFX(moduleConstants.driveMotorID, "canivore1");
    Phoenix6Util.checkErrorAndRetry(() -> mDriveMotor.getConfigurator()
        .apply(SwerveConstants.DriveFXConfig(SwerveConstants.driveMotorInvert), RobotConstants.kLongCANTimeoutS));
    mDriveMotor.setPosition(0.0);

    resetToAbsolute();
    mSignals[0] = mDriveMotor.getRotorPosition();
    mSignals[1] = mDriveMotor.getRotorVelocity();
    mSignals[2] = mAngleMotor.getRotorPosition();
    mSignals[3] = mAngleMotor.getRotorVelocity();
  }

  @Override
  public void readPeriodicInputs() {
    mInputs.timestamp = Timer.getTimestamp();
    refreshSignals();

    Logger.processInputs("Drive/Module" + kModuleNumber, mInputs);
  }

  public void refreshSignals() {
    if (RobotMode.isSim()) {
      mInputs.driveVelocity = mOutputs.driveVelocity;
      mInputs.drivePosition = 0;
      mInputs.rotationPosition = mOutputs.rotTarget;
    } else {
      mInputs.driveVelocity = mDriveMotor.getRotorVelocity().getValue().in(RotationsPerSecond);
      mInputs.drivePosition = mDriveMotor.getRotorPosition().getValueAsDouble();
      mInputs.rotationPosition = Util.placeInAppropriate0To360Scope(0,
          BaseStatusSignal.getLatencyCompensatedValue(
              mAngleMotor.getRotorPosition(), mAngleMotor.getRotorVelocity()).in(Rotation));
      mInputs.absolutePosition = getCanCoder().getDegrees();
    }
  }

  public void setOpenLoop(SwerveModuleState desiredState) {
    double flip = setSteeringAngleOptimized(new Rotation2d(desiredState.angle)) ? -1 : 1;
    mOutputs.driveVelocity = desiredState.speedMetersPerSecond * flip;
    double rotorSpeed = Conversions.MPSToRPS(
        mOutputs.driveVelocity, SwerveConstants.wheelCircumference, SwerveConstants.driveGearRatio);
    mOutputs.driveDemand = rotorSpeed * SwerveConstants.kV;
    mOutputs.driveType = DriveType.OPENLOOP;
  }

  public void setVelocity(SwerveModuleState desiredState) {
    double flip = setSteeringAngleOptimized(new Rotation2d(desiredState.angle)) ? -1 : 1;
    mOutputs.driveVelocity = desiredState.speedMetersPerSecond * flip;
    mOutputs.driveVelocity = Conversions.MPSToRPS(
        mOutputs.driveVelocity,
        com.team5817.frc2025.subsystems.Drive.SwerveConstants.wheelCircumference,
        com.team5817.frc2025.subsystems.Drive.SwerveConstants.driveGearRatio);

    if (Math.abs(mOutputs.driveVelocity) < 0.002) {
      mOutputs.driveType = DriveType.NUETRAL;
    } else {
      mOutputs.driveType = DriveType.VELOCITY;
    }
  }

  private boolean setSteeringAngleOptimized(Rotation2d steerAngle) {
    boolean flip = false;
    final double targetClamped = steerAngle.getDegrees();
    final double angleUnclamped = getCurrentUnboundedDegrees();
    final Rotation2d angleClamped = Rotation2d.fromDegrees(angleUnclamped);
    final Rotation2d relativeAngle = Rotation2d.fromDegrees(targetClamped).rotateBy(angleClamped.inverse());
    double relativeDegrees = relativeAngle.getDegrees();
    if (relativeDegrees > 90.0) {
      relativeDegrees -= 180.0;
      flip = true;

    } else if (relativeDegrees < -90.0) {
      relativeDegrees += 180.0;
      flip = true;
    }
    setSteeringAngleRaw(angleUnclamped + relativeDegrees);
    target_angle = angleUnclamped + relativeDegrees;
    return flip;
  }

  @SuppressWarnings("unused")
  private double target_angle;

  private void setSteeringAngleRaw(double angleDegrees) {
    double rotorPosition = Conversions.degreesToRotation(angleDegrees, SwerveConstants.angleGearRatio);
    mOutputs.rotTarget = rotorPosition;
  }

  @Override
  public void writePeriodicOutputs() {

    mAngleMotor.setControl(new PositionDutyCycle(mOutputs.rotTarget).withVelocity(0).withEnableFOC(true)
        .withFeedForward(0).withSlot(0).withOverrideBrakeDurNeutral(false).withLimitForwardMotion(false)
        .withLimitReverseMotion(false));
    if (mOutputs.driveType == DriveType.OPENLOOP)
      mDriveMotor.setControl(new VoltageOut(mOutputs.driveDemand).withEnableFOC(true));
    else if (mOutputs.driveType == DriveType.VELOCITY)
      mDriveMotor.setControl(new VelocityVoltage(mOutputs.driveVelocity).withFeedForward(0).withEnableFOC(true)
          .withOverrideBrakeDurNeutral(false));

  }

  public void resetToAbsolute() {
    angleEncoder.getAbsolutePosition().waitForUpdate(RobotConstants.kLongCANTimeoutS);
    double angle = Util.placeInAppropriate0To360Scope(
        getCurrentUnboundedDegrees(), getCanCoder().getDegrees() - kAngleOffset);
    double absolutePosition = Conversions.degreesToRotation(angle, SwerveConstants.angleGearRatio);
    Phoenix6Util.checkErrorAndRetry(() -> mAngleMotor.setPosition(absolutePosition, RobotConstants.kLongCANTimeoutS));
  }

  public void setDriveNeutralBrake(boolean wantBrake) {
    TalonFXConfiguration t = new TalonFXConfiguration();
    mDriveMotor.getConfigurator().refresh(t);
    t.MotorOutput.NeutralMode = wantBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    mDriveMotor.getConfigurator().apply(t);
    mAngleMotor.getConfigurator().refresh(t);
    t.MotorOutput.NeutralMode = !wantBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    mAngleMotor.getConfigurator().apply(t);
  }

  @Override
  public void outputTelemetry() {
    // spotless:off
    // spotless:on
  }

  public int moduleNumber() {
    return kModuleNumber;
  }

  public double angleOffset() {
    return kAngleOffset;
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition().getValue().in(Degree));
  }

  public Rotation2d getModuleAngle() {
    return Rotation2d.fromDegrees(getCurrentUnboundedDegrees());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getCurrentVelocity(), getModuleAngle());
  }

  public edu.wpi.first.math.kinematics.SwerveModuleState getWpiState() {
    return new edu.wpi.first.math.kinematics.SwerveModuleState(getCurrentVelocity(),
        edu.wpi.first.math.geometry.Rotation2d.fromDegrees(mInputs.rotationPosition));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDriveDistanceMeters(), getModuleAngle());
  }

  public edu.wpi.first.math.kinematics.SwerveModulePosition getWpiPosition() {
    return new edu.wpi.first.math.kinematics.SwerveModulePosition(
        getDriveDistanceMeters(),
        edu.wpi.first.math.geometry.Rotation2d.fromDegrees(
            getModuleAngle().getDegrees()));
  }

  public double getTargetVelocity() {
    return mOutputs.driveVelocity;
  }

  public double getCurrentVelocity() {
    return Conversions.RPSToMPS(
        mInputs.driveVelocity,
        com.team5817.frc2025.subsystems.Drive.SwerveConstants.wheelCircumference,
        com.team5817.frc2025.subsystems.Drive.SwerveConstants.driveGearRatio);
  }

  public double getDriveDistanceMeters() {
    return Conversions.rotationsToMeters(
        mInputs.drivePosition,
        com.team5817.frc2025.subsystems.Drive.SwerveConstants.wheelCircumference,
        com.team5817.frc2025.subsystems.Drive.SwerveConstants.driveGearRatio);
  }

  public double getCurrentUnboundedDegrees() {
    return Conversions.rotationsToDegrees(mInputs.rotationPosition, SwerveConstants.angleGearRatio);
  }

  public double getTimestamp() {
    return mInputs.timestamp;
  }

  public double getDriveMotorCurrent() {
    return mDriveMotor.getStatorCurrent().getValue().in(Amps);
  }

  public BaseStatusSignal[] getUsedStatusSignals() {
    return mSignals;
  }

  public static class SwerveModuleConstants {
    public final int driveMotorID;
    public final int angleMotorID;
    public final double angleOffset;

    public SwerveModuleConstants(int driveMotorID, int angleMotorID, double angleOffset) {
      this.driveMotorID = driveMotorID;
      this.angleMotorID = angleMotorID;
      this.angleOffset = angleOffset;
    }
  }
}
