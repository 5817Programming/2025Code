package com.team5817.frc2025.subsystems.Vision;

import java.util.List;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

/**
	 * Constants related to the Pose Estimator.
	 */
	public final class VisionConstants {
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
			kSubVisionDevice.kTableName = "limelight-Sub";
		}

	}