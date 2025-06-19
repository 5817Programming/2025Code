package com.team5817.frc2025.subsystems.Vision;

import com.team254.lib.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public interface VisionDeviceIO {
    class VisionDeviceIOInputs {
      public long hb = 0;
      public long fps = -1;
      public boolean is_connected = true;
      public double latency = 0;
      public int tagId = 0;
      public boolean seesTarget = false;
      public double ta = 0;
      public boolean useVision = true;
      public double tagCounts = 0;
      public Pose2d mt2Pose = new Pose2d();
      public Pose2d mt1Pose = new Pose2d();
      public Pose3d targetToCamera = new Pose3d();
    }
  
    default void updateInputs(VisionDeviceIOInputs inputs){

      
    }
  }
  
