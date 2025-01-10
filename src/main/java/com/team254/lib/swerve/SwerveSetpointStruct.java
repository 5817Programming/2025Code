package com.team254.lib.swerve;

import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;
import java.util.Arrays;

public class SwerveSetpointStruct implements Struct<SwerveSetpoint> {

    private static final ChassisSpeedsStruct kChassisSpeedsStruct = new ChassisSpeedsStruct();
    private static final SwerveModuleStateStruct kSwerveModuleStateStruct = new SwerveModuleStateStruct();
    private static final int kNumModules = 4; // Adjust to the number of modules in your system

    @Override
    public Class<SwerveSetpoint> getTypeClass() {
        return SwerveSetpoint.class;
    }

    @Override
    public String getTypeName() {
        return "SwerveSetpoint";
    }

    @Override
    public int getSize() {
        return kChassisSpeedsStruct.getSize() + kNumModules * kSwerveModuleStateStruct.getSize();
    }

    @Override
    public String getSchema() {
        StringBuilder schema = new StringBuilder(kChassisSpeedsStruct.getSchema());
        for (int i = 0; i < kNumModules; i++) {
            schema.append(";").append(kSwerveModuleStateStruct.getSchema());
        }
        return schema.toString();
    }

    @Override
    public SwerveSetpoint unpack(ByteBuffer bb) {
        ChassisSpeeds chassisSpeeds = kChassisSpeedsStruct.unpack(bb);

        SwerveModuleState[] moduleStates = new SwerveModuleState[kNumModules];
        for (int i = 0; i < kNumModules; i++) {
            moduleStates[i] = kSwerveModuleStateStruct.unpack(bb);
        }

        return new SwerveSetpoint(chassisSpeeds, moduleStates);
    }

    @Override
    public void pack(ByteBuffer bb, SwerveSetpoint value) {
        kChassisSpeedsStruct.pack(bb, value.mChassisSpeeds);

        for (SwerveModuleState state : value.mModuleStates) {
            kSwerveModuleStateStruct.pack(bb, state);
        }
    }
}
