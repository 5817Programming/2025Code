package com.team5817.frc2025.subsystems.Vision;

import com.team5817.frc2025.RobotState.VisionUpdate;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

/**
 * VisionDevice class handles vision processing and updates.
 */
public class VisionDevice {
  private final VisionDeviceIO io;
  private final VisionDeviceIO.VisionDeviceIOInputs inputs = new VisionDeviceIO.VisionDeviceIOInputs();
  private Optional<VisionUpdate> visionUpdate = Optional.empty();
  private final String name;

  public VisionDevice(String name, VisionDeviceIO io) {
    this.name = name;
    this.io = io;
  }

  public void update(double timestamp) {
    io.updateInputs(inputs); // Let the IO implementation fill in the data

    // Basic target check: assume MT2 pose is zeroed if no target
    inputs.seesTarget = inputs.mt2Pose.getTranslation().norm() > 0;

    if (inputs.seesTarget) {
      double realTime = timestamp - inputs.latency;
      VisionUpdate update = new VisionUpdate(
        inputs.tagId,
        realTime,
        inputs.ta,
        inputs.mt2Pose.getTranslation()
      );
      visionUpdate = Optional.of(update);


      Logger.recordOutput(name + "/ID", inputs.tagId);
    } else {
      visionUpdate = Optional.empty();
    }

    Logger.recordOutput(name + "/mt1", inputs.mt1Pose.wpi());
  }

  public Optional<VisionUpdate> getVisionUpdate() {
    return visionUpdate;
  }

	/**
	 * Outputs telemetry data.
	 */
	public void outputTelemetry() {
	}


}
