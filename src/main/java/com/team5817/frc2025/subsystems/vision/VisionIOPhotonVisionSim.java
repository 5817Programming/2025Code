// package com.team5817.frc2025.subsystems.Vision;

// import static com.team5817.frc2025.field.FieldLayout.kTagMap;

// import com.team5817.frc2025.subsystems.Vision.VisionDevice;
// import com.team5817.frc2025.subsystems.Vision.VisionDeviceIO;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Transform3d;
// import org.photonvision.simulation.PhotonCameraSim;
// import org.photonvision.simulation.SimCameraProperties;
// import org.photonvision.simulation.VisionSystemSim;

// import java.util.function.Supplier;

// /** IO implementation for physics sim using PhotonVision simulator. */
// public class VisionIOPhotonVisionSim implements VisionDeviceIO {
  
//   private static VisionSystemSim visionSim;
  
//   private final Supplier<Pose2d> poseSupplier;
//   private final PhotonCameraSim cameraSim;

//   /**
//    * Creates a new VisionIOPhotonVisionSim.
//    *
//    * @param name The name of the camera.
//    * @param robotToCamera The transform from the robot to the camera.
//    * @param poseSupplier Supplier for the robot pose to use in simulation.
//    */
//   public VisionIOPhotonVisionSim(String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
//     this.poseSupplier = poseSupplier;

//     // Initialize vision sim
//     if (visionSim == null) {
//       visionSim = new VisionSystemSim("main");
//       visionSim.addAprilTags(kTagMap); // Using your `kTagMap` for AprilTags
//     }

//     // Add sim camera
//     SimCameraProperties cameraProperties = new SimCameraProperties();
//     cameraSim = new PhotonCameraSim(name, cameraProperties, kTagMap);
//     visionSim.addCamera(cameraSim, robotToCamera);
//   }

//   /**
//    * Updates the VisionIOInputs based on the simulation.
//    *
//    * @param inputs The VisionIOInputs instance to update.
//    */
//   @Override
//   public void updateInputs(VisionIOInputs inputs) {
//     // Update vision sim with the current robot pose
//     visionSim.update(poseSupplier.get());

//     // Fetch the latest camera result
//     var result = cameraSim.getLatestResult();
    
//     if (result.hasTargets()) {
//       var best = result.getBestTarget();

//       result.getMultiTagResult().estimatedPose.ifPresent(estimated -> {
//         VisionFrame frame = new VisionFrame(
//           name,
//           estimated.getTimestampSeconds(),
//           estimated.getPose().toPose2d(),
//           best.getFiducialId(),
//           best.getYaw(),
//           best.getPitch()
//         );

//         VisionDeviceManager.addVisionFrame(frame);
//       });
//     }
//   }
// }
