package com.team5817.frc2025.subsystems.Vision;

import org.littletonrobotics.junction.Logger;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Twist2d;
import com.team5817.frc2025.field.FieldLayout;


/**
 * This class is responsible for accepting or rejecting vision-based pose updates.
 */
public class VisionPoseAcceptor {
	private static final double kFieldBorderMargin = 0.5;
	private static final double kMaxVisionCorrection = 2.0; // Jump from fused pose

	Pose2d mLastVisionFieldToVehicle = null;

	/**
	 * Determines whether the vision-based pose update should be accepted.
	 *
	 * @param timestamp The timestamp of the vision update.
	 * @param visionFieldToVehicle The pose of the vehicle based on vision data.
	 * @param lastFieldToVehicle The last known pose of the vehicle.
	 * @param robotVelocity The current velocity of the robot.
	 * @param isInAuto Whether the robot is in autonomous mode.
	 * @return true if the vision update should be accepted, false otherwise.
	 */
	public boolean shouldAcceptVision(
			double timestamp,
			Pose2d visionFieldToVehicle,
			Pose2d lastFieldToVehicle,
			Twist2d robotVelocity,
			boolean isInAuto) {

		// If first update, trust
		if (mLastVisionFieldToVehicle == null) {
			mLastVisionFieldToVehicle = visionFieldToVehicle;
			return true;
		}

		// Write last pose early because we return out of the method
		mLastVisionFieldToVehicle = visionFieldToVehicle;

		// Check out of field
		if (visionFieldToVehicle.getTranslation().x() < -kFieldBorderMargin
				|| visionFieldToVehicle.getTranslation().x() > FieldLayout.kFieldLength + kFieldBorderMargin
				|| visionFieldToVehicle.getTranslation().y() < -kFieldBorderMargin
				|| visionFieldToVehicle.getTranslation().y() > FieldLayout.kFieldWidth + kFieldBorderMargin) {
			Logger.recordOutput("Vision validation", "Outside field");
			return false;
		}

		if (robotVelocity.norm() > 4.0) {
			Logger.recordOutput("Vision validation", "Max velocity");
			return false;
		}

		if (isInAuto) {
			// Check max correction
			if (visionFieldToVehicle.distance(lastFieldToVehicle) > kMaxVisionCorrection) {
				Logger.recordOutput("Vision validation", "Max correction");
				return false;
			}
		}

		Logger.recordOutput("Vision validation", "OK");
		return true;
	}
}
