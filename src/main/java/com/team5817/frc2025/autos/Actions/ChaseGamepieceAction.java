package com.team5817.frc2025.autos.Actions;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.swerve.ChassisSpeeds;
import com.team5817.frc2025.subsystems.Superstructure;
import com.team5817.frc2025.subsystems.Drive.Drive;
import com.team5817.frc2025.subsystems.GamePieceVision.GamepieceVision;

public class ChaseGamepieceAction implements Action{

    private final Drive mDrive;
    private final GamepieceVision mGamepieceVision;
    private final Superstructure mSuperstructure;

    public ChaseGamepieceAction(Drive drive, GamepieceVision gamepieceVision, Superstructure superstructure){
        this.mSuperstructure = superstructure;
        this.mDrive = drive;
        this.mGamepieceVision = gamepieceVision;
    }

    @Override
    public boolean isFinished() {
        return mSuperstructure.hasPiece();
    }

    @Override
    public void update() {
        if (mGamepieceVision.getBestGamepiecePose().isEmpty() || mGamepieceVision.getBestRobotToGamepiecePose().isEmpty())
            return;
       
            mDrive.alignDrive(
            Pose2d.fromTranslation(mGamepieceVision.getBestGamepiecePose().get())
                .withRotation(mGamepieceVision.getBestRobotToGamepiecePose().get().direction().flip()),
            new Pose2d());
    }

    @Override
    public void start() {}

    @Override
    public void done() {
        mDrive.runVelocity(new ChassisSpeeds());
    }
    
}
