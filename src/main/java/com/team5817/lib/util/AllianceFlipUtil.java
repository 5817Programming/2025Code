// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team5817.lib.util;

import com.team5817.frc2025.field.FieldLayout;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;

public class AllianceFlipUtil {
  public static double applyX(double x) {
    return shouldFlip() ? FieldLayout.kFieldLength - x : x;
  }

  public static double applyY(double y) {
    return shouldFlip() ? FieldLayout.kFieldWidth - y : y;
  }

  public static Translation2d apply(Translation2d translation) {
    return new Translation2d(applyX(translation.getX()), applyY(translation.getY()));
  }

  public static Rotation2d apply(Rotation2d rotation) {
    return shouldFlip() ? rotation.rotateBy(Rotation2d.kPi) : rotation;
  }

  public static Pose2d apply(Pose2d pose) {
    return shouldFlip()
        ? new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()))
        : pose;
  }

  public static Translation3d apply(Translation3d translation) {
    return new Translation3d(
        applyX(translation.getX()), applyY(translation.getY()), translation.getZ());
  }

  public static Rotation3d apply(Rotation3d rotation) {
    return shouldFlip() ? rotation.rotateBy(new Rotation3d(0.0, 0.0, Math.PI)) : rotation;
  }

  public static Pose3d apply(Pose3d pose) {
    return new Pose3d(apply(pose.getTranslation()), apply(pose.getRotation()));
  }


  public static boolean shouldFlip() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
  }

    /**
   * Explicitly flip a pose across the X and/or Y axis of the field, regardless of alliance.
   */
  public static Pose3d apply(Pose3d pose, boolean flipX, boolean flipY) {
    double x = flipX ? FieldLayout.kFieldLength - pose.getX() : pose.getX();
    double y = flipY ? FieldLayout.kFieldWidth - pose.getY() : pose.getY();
    Rotation3d rotation = pose.getRotation();

    if (flipX ^ flipY) {
      // Flip rotation 180 degrees around Z if flipping across one axis
      rotation = rotation.rotateBy(new Rotation3d(0, 0, Math.PI));
    }

    return new Pose3d(new Translation3d(x, y, pose.getZ()), rotation);
  }
}