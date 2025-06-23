package com.team5817.frc2025.autos.Actions;

import com.team254.lib.swerve.ChassisSpeeds;
import com.team5817.frc2025.subsystems.Superstructure;
import com.team5817.frc2025.subsystems.Drive.Drive;
import com.team5817.frc2025.subsystems.GamePieceVision.GamepieceVision;
import com.team5817.lib.motion.Trajectory;

public class TrackingTrajectoryAction implements Action{
    final TrajectoryAction trajectoryAction;
    final ChaseGamepieceAction chaseGamepieceAction;
    final GamepieceVision mGamepieceVision;
    final Drive mDrive;
    boolean tracking = false;

  public TrackingTrajectoryAction(Trajectory path, Drive drive, GamepieceVision gamepieceVision, Superstructure superstructure) {
    this.mDrive = drive;
    this.mGamepieceVision = gamepieceVision;
    this.chaseGamepieceAction = new ChaseGamepieceAction(drive, this.mGamepieceVision, superstructure);
    this.trajectoryAction = new TrajectoryAction(path, false, Double.MAX_VALUE, drive);
  }

  public TrackingTrajectoryAction(Trajectory path, boolean resetPos, Drive drive, GamepieceVision gamepieceVision, Superstructure superstructure) {
    this.mDrive = drive;
    this.mGamepieceVision = gamepieceVision;
    this.chaseGamepieceAction = new ChaseGamepieceAction(drive, this.mGamepieceVision, superstructure);
    this.trajectoryAction = new TrajectoryAction(path, resetPos, Double.MAX_VALUE, drive);
  }

  public TrackingTrajectoryAction(Trajectory path, double extraTimeout, Drive drive, GamepieceVision gamepieceVision, Superstructure superstructure) {
    this.mDrive = drive;
    this.mGamepieceVision = gamepieceVision;
    this.chaseGamepieceAction = new ChaseGamepieceAction(drive, this.mGamepieceVision, superstructure);
    this.trajectoryAction = new TrajectoryAction(path, false, extraTimeout, drive);
  }

@Override
public boolean isFinished() {
    if(tracking)
      return chaseGamepieceAction.isFinished();
    else 
      return trajectoryAction.isFinished();
}

@Override
public void update() {
    if(mGamepieceVision.getBestGamepiecePose().isPresent()&&!tracking){
        tracking = true;
        System.out.println("Tracking piece");
        mDrive.runVelocity(new ChassisSpeeds());
        chaseGamepieceAction.start();
      }

    if(!tracking)
        trajectoryAction.update();
    else
        chaseGamepieceAction.update();
}

@Override
public void done() {
  if(tracking)
    chaseGamepieceAction.done();
  else
    trajectoryAction.done();
}

@Override
public void start() {
  trajectoryAction.start();
}
  
}