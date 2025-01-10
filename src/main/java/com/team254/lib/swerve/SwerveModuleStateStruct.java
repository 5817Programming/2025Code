// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team254.lib.swerve;

import edu.wpi.first.util.struct.Struct;


import java.nio.ByteBuffer;

import com.team254.lib.geometry.Rotation2d;

public class SwerveModuleStateStruct implements Struct<SwerveModuleState> {
  @Override
  public Class<SwerveModuleState> getTypeClass() {
    return SwerveModuleState.class;
  }

  @Override
  public String getTypeName() {
    return "SwerveModuleState";
  }

  @Override
  public int getSize() {
    return kSizeDouble * 3;
  }

  @Override
  public String getSchema() {
    return "double speedMetersPerSecond;double distanceMeters;Rotation2d angle";
  }

  @Override
  public SwerveModuleState unpack(ByteBuffer bb) {
    double speedMetersPerSecond = bb.getDouble();
    double distanceMeters = bb.getDouble();
    Rotation2d angle = Rotation2d.struct.unpack(bb);
    return new SwerveModuleState(speedMetersPerSecond,distanceMeters,angle);
  }

  @Override
  public void pack(ByteBuffer bb, SwerveModuleState value) {
    bb.putDouble(value.speedMetersPerSecond);
    bb.putDouble(value.distanceMeters);
    Rotation2d.struct.pack(bb, value.angle);
}
}
