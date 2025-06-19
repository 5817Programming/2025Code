package com.team5817.frc2025.subsystems.Vision;

import com.team5817.frc2025.RobotConstants;
import com.team5817.frc2025.RobotState;
import com.team5817.frc2025.RobotState.VisionUpdate;
import com.team5817.frc2025.loops.ILooper;
import com.team5817.frc2025.loops.Loop;
import com.team5817.frc2025.subsystems.Drive.Drive;
import com.team5817.lib.RobotMode;
import com.team5817.lib.drivers.Subsystem;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import java.util.List;

import org.littletonrobotics.junction.Logger;

/**
 * Manages vision devices and processes vision data.
 */
public class VisionDeviceManager extends Subsystem {

  private VisionDevice mRightCamera;
  private VisionDevice mLeftCamera;
  private VisionDevice mUpCamera;

  private List<VisionDevice> mAllCameras;

  private static double timestampOffset = 0.1;

  private static boolean disable_vision = false;
  double timeOfLastUpdate = Double.MIN_VALUE;

  /**
   * Constructor for VisionDeviceManager.
   */

  public VisionDeviceManager(VisionDeviceIO FrontLeft, VisionDeviceIO FrontRight, VisionDeviceIO Up) {
    mRightCamera = new VisionDevice("limelight-right", FrontRight);
    mLeftCamera = new VisionDevice("limelight-left", FrontLeft);
    mUpCamera = new VisionDevice("limelight-up", Up);

    mAllCameras = List.of(mRightCamera, mLeftCamera, mUpCamera);
  }

  /**
   * Registers the enabled loops.
   * 
   * @param enabledLooper the enabled looper.
   */
  @Override
  public void registerEnabledLoops(ILooper enabledLooper) {
    enabledLooper.register(new Loop() {
      @Override
      public void onStart(double timestamp) {
      }

      @Override
      public void onLoop(double timestamp) {

      }

    });
  }

  /**
   * Reads periodic inputs from vision devices.
   */
  @Override
  public void readPeriodicInputs() {
    if (RobotMode.isSim()) {
      RobotState.getInstance().addVisionUpdate(new VisionUpdate(1, Timer.getTimestamp(), 1.0,
          RobotState.getInstance().getPoseFromOdom(Timer.getTimestamp()).getTranslation()));
      return;
    }
    for (VisionDevice device : mAllCameras)
      updateDevice(device);

  }

  public void updateDevice(VisionDevice device) {
    device.update(Timer.getTimestamp());
    if (!device.getVisionUpdate().isEmpty()) {
      VisionUpdate update = device.getVisionUpdate().get();
      RobotState.getInstance().addVisionUpdate(update);
      if (update.getTimestamp() > timeOfLastUpdate)
        timeOfLastUpdate = update.getTimestamp();
    }
  }

  /**
   * Writes periodic outputs.
   */
  @Override
  public void writePeriodicOutputs() {
  }

  /**
   * Outputs telemetry data to the dashboard.
   */
  @Override
  public void outputTelemetry() {
    Logger.recordOutput("Elastic/Time Since Last Update", Timer.getTimestamp() - timeOfLastUpdate);
    for (VisionDevice device : mAllCameras) {
      device.outputTelemetry();
    }
  }

  /**
   * Returns the best vision device based on the vision update.
   * 
   * @return the best vision device.
   */
  public VisionDevice getBestDevice() {
    double bestTa = 0;
    VisionDevice bestDevice = null;
    for (VisionDevice device : mAllCameras) {
      if (device.getVisionUpdate().isEmpty()) {
        continue;
      }
      if (device.getVisionUpdate().get().getTa() > bestTa) {
        bestTa = device.getVisionUpdate().get().getTa();
        bestDevice = device;
      }
    }
    return bestDevice;

  }

  // **constructor**
  public VisionDeviceManager(VisionDevice right, VisionDevice left, VisionDevice up) {
    mRightCamera = right;
    mLeftCamera = left;
    mUpCamera = up;
    mAllCameras = List.of(mRightCamera, mLeftCamera, mUpCamera);
  }

  /**
   * Verifies epipolar geometry between two sets of points.
   * 
   * @param pointsCam1 points from the first camera.
   * @param pointsCam2 points from the second camera.
   * @return true if the points satisfy the epipolar constraint, false otherwise.
   */
  public boolean epipolarVerification(List<Translation2d> pointsCam1, List<Translation2d> pointsCam2) {

    if (pointsCam1.size() != pointsCam2.size()) {
      throw new IllegalArgumentException("Point lists must have the same size.");
    }
    double threshold = 1e-6; // Tolerance for numerical errors
    for (int i = 0; i < pointsCam1.size(); i++) {
      Translation3d x1 = toHomogeneous(pointsCam1.get(i));
      Translation3d x2 = toHomogeneous(pointsCam2.get(i));
      // Compute Fx1 = F * x1
      Translation3d Fx1 = multiplyMatrixVector(RobotConstants.fundamentalMatrix, x1);
      // Compute x2 • (Fx1)
      double result = x2.dot(Fx1);
      // Check if result is close to zero
      if (Math.abs(result) > threshold) {
        return false;
      }
    }
    return true;

  }

  /**
   * Returns the left vision device.
   * 
   * @return the left vision device.
   */
  public VisionDevice getLeftVision() {
    return mRightCamera;
  }

  /**
   * Returns the right vision device.
   * 
   * @return the right vision device.
   */
  public VisionDevice getRightVision() {
    return mLeftCamera;
  }

  /**
   * Returns the timestamp offset.
   * 
   * @return the timestamp offset.
   */
  public static double getTimestampOffset() {
    return timestampOffset;
  }

  /**
   * Checks if vision is disabled.
   * 
   * @return true if vision is disabled, false otherwise.
   */
  public static boolean visionDisabled() {
    return disable_vision;
  }

  /**
   * Sets the vision disabled state.
   * 
   * @param disable the new state of vision disabled.
   */
  public static void setDisableVision(boolean disable) {
    disable_vision = disable;
  }

  /**
   * Verifies epipolar geometry between two sets of points using a fundamental
   * matrix.
   * 
   * @param pointsCam1        points from the first camera.
   * @param pointsCam2        points from the second camera.
   * @param fundamentalMatrix the fundamental matrix.
   * @return true if the points satisfy the epipolar constraint, false otherwise.
   */
  public static boolean verifyEpipolarGeometry(List<Translation2d> pointsCam1,
      List<Translation2d> pointsCam2,
      double[][] fundamentalMatrix) {
    if (pointsCam1.size() != pointsCam2.size()) {
      throw new IllegalArgumentException("Point lists must have the same size.");
    }

    double threshold = 1e-6; // Tolerance for numerical errors
    for (int i = 0; i < pointsCam1.size(); i++) {
      Translation3d x1 = toHomogeneous(pointsCam1.get(i));
      Translation3d x2 = toHomogeneous(pointsCam2.get(i));

      // Compute Fx1 = F * x1
      Translation3d Fx1 = multiplyMatrixVector(fundamentalMatrix, x1);

      // Compute x2 • (Fx1)
      double result = x2.dot(Fx1);

      // Check if result is close to zero
      if (Math.abs(result) > threshold) {
        return false;
      }
    }

    return true;
  }

  /**
   * Converts a Translation2D to homogeneous coordinates (Translation3D).
   * 
   * @param point the 2D point.
   * @return the 3D point in homogeneous coordinates.
   */
  private static Translation3d toHomogeneous(Translation2d point) {
    return new Translation3d(point.x(), point.y(), 1.0);
  }

  /**
   * Multiplies a 3x3 matrix with a 3D vector.
   * 
   * @param matrix the 3x3 matrix.
   * @param vector the 3D vector./
   * @return the resulting 3D vector.
   */
  private static Translation3d multiplyMatrixVector(double[][] matrix, Translation3d vector) {
    double x = matrix[0][0] * vector.x() + matrix[0][1] * vector.y() + matrix[0][2] * vector.z();
    double y = matrix[1][0] * vector.x() + matrix[1][1] * vector.y() + matrix[1][2] * vector.z();
    double z = matrix[2][0] * vector.x() + matrix[2][1] * vector.y() + matrix[2][2] * vector.z();
    return new Translation3d(x, y, z);
  }

}
