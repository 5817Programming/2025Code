package com.team5817.frc2025.autos.Actions;

import com.team254.lib.geometry.Pose2d;
import com.team5817.frc2025.field.AlignmentPoint.AlignmentType;
import com.team5817.frc2025.subsystems.Drive.Drive;

public class AutoAlignAction implements Action{
    private final Drive mDrive;
    private final Pose2d targetPose;
    private final Pose2d tolerance;
    public AutoAlignAction(Drive d, Pose2d targetPose, Pose2d tolerance){
        this.mDrive = d;
        this.targetPose = targetPose;
        this.tolerance = tolerance;
    }
    public AutoAlignAction(Drive d, AlignmentType alignmentType){
        this.mDrive = d;
        this.tolerance = alignmentType.tolerance;

        d.setAlignment(alignmentType);
        this.targetPose = d.findTargetPoint();
    }
    @Override
    public boolean isFinished() {
        return mDrive.getAutoAlignComplete();    
    }

    @Override
    public void update() {}

    @Override
    public void done() {}

    @Override
    public void start() {
        mDrive.alignDrive(targetPose, tolerance);
    }
    
}
