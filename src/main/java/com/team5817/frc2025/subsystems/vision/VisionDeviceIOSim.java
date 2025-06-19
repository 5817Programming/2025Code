package com.team5817.frc2025.subsystems.Vision;

import com.team254.lib.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;


public class VisionDeviceIOSim implements VisionDeviceIO{
    private final VisionDeviceIOInputs inputs = new VisionDeviceIOInputs();
   
    public VisionDeviceIOSim() {
        // example data for sims
        inputs.seesTarget = true;
        inputs.tagId = 1;
        inputs.ta = 3.14;
        inputs.latency = 0.04;
        inputs.fps = 30;
        inputs.tagCounts = 1;
        inputs.mt1Pose = new Pose2d(1.0, 2.0, 0.0);
        inputs.mt2Pose = new Pose2d(1.5, 2.5, 0.0);
        inputs.targetToCamera = new Pose3d();
      }


      
  @Override
  public void updateInputs(VisionDeviceIOInputs inputsToUpdate) {
    inputsToUpdate.seesTarget = inputs.seesTarget;
    inputsToUpdate.tagId = inputs.tagId;
    inputsToUpdate.ta = inputs.ta;
    inputsToUpdate.latency = inputs.latency;
    inputsToUpdate.fps = inputs.fps;
    inputsToUpdate.tagCounts = inputs.tagCounts;
    inputsToUpdate.mt1Pose = inputs.mt1Pose;
    inputsToUpdate.mt2Pose = inputs.mt2Pose;
    inputsToUpdate.targetToCamera = inputs.targetToCamera;
  }
}
