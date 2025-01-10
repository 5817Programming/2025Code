package com.team254.lib.swerve;

import edu.wpi.first.util.struct.StructSerializable;

public class SwerveSetpoint implements StructSerializable {
    public ChassisSpeeds mChassisSpeeds;
    public SwerveModuleState[] mModuleStates;

    public SwerveSetpoint(ChassisSpeeds chassisSpeeds, SwerveModuleState[] initialStates) {
        this.mChassisSpeeds = chassisSpeeds;
        this.mModuleStates = initialStates;
    }

    @Override
    public String toString() {
        String ret = mChassisSpeeds.toString() + "\n";
        for (int i = 0; i < mModuleStates.length; ++i ) {
            ret += "  " + mModuleStates[i].toString() + "\n";
        }
        return ret;
    }
    public static final SwerveSetpointStruct struct = new SwerveSetpointStruct();
}
