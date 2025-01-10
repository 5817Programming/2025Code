package com.team5817.frc2024.subsystems.vision;

import com.team254.lib.geometry.Translation2d;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionDeviceConstants {
	public String kTableName = "";
	public Transform3d kRobotToCamera = new Transform3d();
	public int kCameraId = 0;
	public int kCameraResolutionWidth = 1600;
	public int kCameraResolutionHeight = 1200;
}
