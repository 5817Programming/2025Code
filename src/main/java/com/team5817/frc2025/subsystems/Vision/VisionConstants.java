package com.team5817.frc2025.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "camera_0";
  public static String camera1Name = "camera_1";



  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToCameraUp =
    new Transform3d(.0005, 0., 1.0115, new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-21), Units.degreesToRadians(180)));
  public static Transform3d robotToCameraLeft =
      new Transform3d(.133, 0.2642, 0.2024, new Rotation3d(Units.degreesToRadians(-1.5), Units.degreesToRadians(-20.5), Units.degreesToRadians(-10.5)));
  public static Transform3d robotToCameraRight =
    new Transform3d(.133, -0.2642, 0.2024, new Rotation3d(Units.degreesToRadians(1.5), Units.degreesToRadians(-20.5), Units.degreesToRadians(10.5)));
  public static Transform3d robotToCameraBack =
    new Transform3d(.0005, 0., 1.0115, new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(21), Units.degreesToRadians(180)));
  
  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0, // Camera 1
        1.0 // Camera 2
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
