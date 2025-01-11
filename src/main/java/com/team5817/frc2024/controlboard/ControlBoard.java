package com.team5817.frc2024.controlboard;

import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team5817.frc2024.Constants;
import com.team5817.frc2024.subsystems.Drive;
import com.team5817.lib.Util;

import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ControlBoard {
	private final double kSwerveDeadband = Constants.stickDeadband;

	private static ControlBoard mInstance = null;

	public static ControlBoard getInstance() {
		if (mInstance == null) {
			mInstance = new ControlBoard();
		}

		return mInstance;
	}

	public final CustomXboxController driver;
	public final CustomXboxController operator;

	private ControlBoard() {
		driver = new CustomXboxController(0);
		operator = new CustomXboxController(Constants.kButtonGamepadPort);
	}

	public void update() {
		driver.update();
		operator.update();
	}

	/* DRIVER METHODS */
	public Translation2d getSwerveTranslation() {
		double forwardAxis = driver.getRawAxis(Axis.kLeftY.value);
		double strafeAxis = driver.getRawAxis(Axis.kLeftX.value);

		double expoForwardAxis = Math.pow(forwardAxis, 3) ;
		double expoStrafeAxis = Math.pow(strafeAxis, 3) ;

		SmartDashboard.putNumber("Raw Y", forwardAxis);
		SmartDashboard.putNumber("Raw X", strafeAxis);

		expoForwardAxis = Constants.SwerveConstants.invertYAxis ? expoForwardAxis : -expoForwardAxis;
		expoStrafeAxis = Constants.SwerveConstants.invertXAxis ? expoStrafeAxis : -expoStrafeAxis;
		
		Translation2d tAxes = new Translation2d(expoForwardAxis, expoStrafeAxis);

		if (Math.abs(tAxes.norm()) < kSwerveDeadband) {
			return new Translation2d();
		} else {
			Rotation2d deadband_direction = new Rotation2d(tAxes.x(), tAxes.y(), true);
			Translation2d deadband_vector = Translation2d.fromPolar(deadband_direction, kSwerveDeadband);

			double scaled_x = Util.scaledDeadband(expoForwardAxis, 1.0, Math.abs(deadband_vector.x()));
			double scaled_y = Util.scaledDeadband(expoStrafeAxis, 1.0, Math.abs(deadband_vector.y()));
			return new Translation2d(scaled_x, scaled_y)
					.scale(Drive.getInstance().getKinematicLimits().kMaxDriveVelocity);
		}
	}

	public double getSwerveRotation() {
		double rotAxis = driver.getRightX() * 0.80;
		rotAxis = Constants.SwerveConstants.invertRAxis ? rotAxis : -rotAxis;

		if (Math.abs(rotAxis) < kSwerveDeadband) {
			return 0.0;
		} else {
			return Drive.getInstance().getKinematicLimits().kMaxSteeringVelocity
					* (rotAxis - (Math.signum(rotAxis) * kSwerveDeadband))
					/ (1 - kSwerveDeadband);
		}
	}

	public boolean zeroGyro() {
		return driver.startButton.isBeingPressed() && driver.backButton.isBeingPressed();
	}

	// Only Driver
	public boolean topButtonsClearDriver() {
		return !(driver.leftBumper.isBeingPressed()
				|| driver.rightBumper.isBeingPressed()
				|| driver.leftTrigger.isBeingPressed()
				|| driver.rightTrigger.isBeingPressed());
	}


	// Driver and Operator

	public boolean topButtonsClearOperator() {
		return !(operator.leftBumper.isBeingPressed()
				|| operator.rightBumper.isBeingPressed()
				|| operator.leftTrigger.isBeingPressed()
				|| operator.rightTrigger.isBeingPressed());
	}

}
