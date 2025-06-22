package com.team5817.frc2025.subsystems.GamePieceVision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

/** IO implementation for detecting gamepieces using Limelight and tx/ty directly. */
public class GamepieceVisionIOLimelight implements GamepieceVisionIO {
  private final DoubleSubscriber txSubscriber;
  private final DoubleSubscriber tySubscriber;
  private final DoubleSubscriber latencySubscriber;

  private final Transform3d robotToCamera;

  // Heights in meters
  private final double cameraHeightMeters;
  private final double targetHeightMeters;

  // Camera mounting pitch angle in radians (e.g., 30 degrees down is -30 deg)
  private final double cameraPitchRadians;

  public GamepieceVisionIOLimelight(
      String limelightName,
      Transform3d robotToCamera,
      double targetHeightMeters) {
    var table = NetworkTableInstance.getDefault().getTable("limelight-" + limelightName);
    txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
    tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
    latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);

    this.robotToCamera = robotToCamera;
    this.cameraHeightMeters = robotToCamera.getZ();
    this.targetHeightMeters = targetHeightMeters;
    this.cameraPitchRadians = robotToCamera.getRotation().getY();
  }

  @Override
  public void updateInputs(GamepieceVisionIOInputs inputs) {

    double tx = txSubscriber.get();
    double ty = tySubscriber.get();
    double latencySec = latencySubscriber.get() / 1000.0;
    double timestamp = Timer.getTimestamp() - latencySec;

    // Convert Limelight angles to radians
    double pitchRadians = Units.degreesToRadians(ty);
    double yawRadians = Units.degreesToRadians(tx);

    // Compute distance to target using vertical geometry:
    // distance = (targetHeight - cameraHeight) / tan(cameraPitch + pitch)
    double verticalAngle = cameraPitchRadians + pitchRadians;
    if (Math.abs(verticalAngle) < 1e-6) {
      // Avoid divide by zero or very small angle
      return;
    }
    double distance = (targetHeightMeters - cameraHeightMeters) / Math.tan(verticalAngle);

    // Now build the 3D vector from camera frame to target:
    Translation3d cameraToTarget = new Translation3d(
        distance * Math.cos(pitchRadians) * Math.sin(yawRadians), // X (left-right)
        distance * Math.sin(pitchRadians),                         // Y (up-down)
        distance * Math.cos(pitchRadians) * Math.cos(yawRadians)  // Z (forward)
    );

    // Transform to robot frame
    Translation3d robotToTarget = robotToCamera.plus(
        new Transform3d(cameraToTarget, new Rotation3d())
    ).getTranslation();

    inputs.data = new GamepieceVisionIOData(robotToTarget, timestamp);
  }
}
