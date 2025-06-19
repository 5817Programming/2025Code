package com.team5817.frc2025.subsystems.Vision;

import com.team254.lib.geometry.Pose2d;

/**
 * Represents a frame of vision data with a timestamp.
 */
public class VisionFrame {
	public final String cameraName;
	public final double timestamp;
	public final Pose2d estimatedPose;
	public final int tagId;
	public final double tx;
	public final double ty;
  
	public VisionFrame(String cameraName, double timestamp, Pose2d estimatedPose, int tagId, double tx, double ty) {
	  this.cameraName = cameraName;
	  this.timestamp = timestamp;
	  this.estimatedPose = estimatedPose;
	  this.tagId = tagId;
	  this.tx = tx;
	  this.ty = ty;
	}
  }
  