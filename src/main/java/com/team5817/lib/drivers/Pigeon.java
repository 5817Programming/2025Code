package com.team5817.lib.drivers;

import static edu.wpi.first.units.Units.Degree;


import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import com.team254.lib.geometry.Rotation2d;
import com.team5817.frc2025.Constants;
import com.team5817.frc2025.Ports;
import com.team5817.frc2025.Robot;

public class Pigeon {

	private static Pigeon mInstance;

	public static Pigeon getInstance() {
		if (mInstance == null) {
			mInstance = new Pigeon(Ports.PIGEON);
		}
		return mInstance;
	}

	// Actual pigeon object
	private final Pigeon2 mGyro;

	// Configs
	private boolean inverted = Constants.SwerveConstants.invertGyro;
	private Rotation2d yawAdjustmentAngle = new Rotation2d();
	private Rotation2d rollAdjustmentAngle = new Rotation2d();
	private Rotation2d pitchAdjustmentAngle = new Rotation2d();

	private Pigeon(int port) {
		mGyro = new Pigeon2(port, "canivore1");
		mGyro.getConfigurator().apply(new Pigeon2Configuration());
	}

	static SwerveDriveSimulation driveSim;

	public static void registerGyroSim(SwerveDriveSimulation sim) {
		driveSim = sim;
	}

	public Rotation2d getYaw() {
		if (!Robot.isReal() && Constants.mode == Constants.Mode.SIM) {
			return Rotation2d.fromDegrees(driveSim.getSimulatedDriveTrainPose().getRotation().getDegrees());
		}
		Rotation2d angle = getUnadjustedYaw().rotateBy(yawAdjustmentAngle.inverse());
		if (inverted) {
			return angle.inverse();
		}
		return angle;
	}

	public Rotation2d getRoll() {
		return getUnadjustedRoll().rotateBy(rollAdjustmentAngle.inverse());
	}

	public Rotation2d getPitch() {
		if (!Robot.isReal() && Constants.mode == Constants.Mode.SIM) {
			return Rotation2d.identity();
		}
		return getUnadjustedPitch().rotateBy(pitchAdjustmentAngle.inverse()).inverse();
	}

	/**
	 * Sets the yaw register to read the specified value.
	 *
	 * @param angleDeg New yaw in degrees
	 */
	public void setYaw(double angleDeg) {
		yawAdjustmentAngle = Rotation2d.fromDegrees(getYawStatusSignal().getValueAsDouble())
				.rotateBy(Rotation2d.fromDegrees(angleDeg).inverse());
	}

	/**
	 * Sets the roll register to read the specified value.
	 *
	 * @param angleDeg New yaw in degrees
	 */
	public void setRoll(double angleDeg) {
		rollAdjustmentAngle = getUnadjustedRoll().rotateBy(Rotation2d.fromDegrees(angleDeg).inverse());
	}

	/**
	 * Sets the roll register to read the specified value.
	 *
	 * @param angleDeg New yaw in degrees
	 */
	public void setPitch(double angleDeg) {
		pitchAdjustmentAngle = getUnadjustedPitch().rotateBy(Rotation2d.fromDegrees(angleDeg).inverse());
		System.out.println("Reset gyro to " + getPitch().getDegrees());
	}

	public Rotation2d getUnadjustedYaw() {

		return Rotation2d.fromDegrees(
				BaseStatusSignal.getLatencyCompensatedValue(getYawStatusSignal(), getRateStatusSignal()).in(Degree));
	}

	public Rotation2d getUnadjustedPitch() {
		return Rotation2d.fromDegrees(mGyro.getRoll().getValue().in(Degree));
	}

	public Rotation2d getUnadjustedRoll() {
		return Rotation2d.fromDegrees(mGyro.getPitch().getValue().in(Degree));
	}

	public StatusSignal<Angle> getYawStatusSignal() {
		return mGyro.getYaw();
	}

	public StatusSignal<AngularVelocity> getRateStatusSignal() {
		return mGyro.getAngularVelocityZDevice();
	}
}
