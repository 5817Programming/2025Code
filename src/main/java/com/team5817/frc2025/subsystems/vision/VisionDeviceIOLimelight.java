package com.team5817.frc2025.subsystems.Vision;


import com.team254.lib.geometry.Pose2d;
import com.team5817.frc2025.subsystems.Drive.Drive;
import com.team5817.frc2025.subsystems.Vision.LimelightHelpers.PoseEstimate;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;


public class VisionDeviceIOLimelight implements VisionDeviceIO{
    private final String name; 
    private final NetworkTable limelightTable;
    private Drive mDrive;

    public VisionDeviceIOLimelight(String name, Drive drive) {
        this.name = name;
        this.mDrive = drive;
        this.limelightTable = NetworkTableInstance.getDefault().getTable(name);
      }

      @Override
      public void updateInputs(VisionDeviceIOInputs inputs) {
        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    
        inputs.latency = limelightTable.getEntry("latency").getDouble(0.0);
        inputs.fps = limelightTable.getEntry("fps").getInteger(0);
        inputs.tagId = limelightTable.getEntry("tid").getNumber(-1).intValue();
        inputs.ta = limelightTable.getEntry("ta").getDouble(0.0);
        inputs.seesTarget = false;
    
        if (estimate != null && estimate.pose.getTranslation().getNorm() != 0 && LimelightHelpers.getTV(name)) {
          inputs.seesTarget = true;
          inputs.tagCounts = estimate.tagCount;
          inputs.mt2Pose = new Pose2d(estimate.pose);
          inputs.mt1Pose = new Pose2d(LimelightHelpers.getBotPose2d_wpiBlue(name));
          inputs.targetToCamera = LimelightHelpers.getTargetPose3d_CameraSpace(name);
        }
        LimelightHelpers.SetRobotOrientation(name, mDrive.getHeading().getDegrees(), 0, 0, 0, 0, 0);
    
}
}