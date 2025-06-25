package com.team5817.lib.swerve;

import org.littletonrobotics.junction.AutoLogOutput;

import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.SynchronousPIDF;

import edu.wpi.first.wpilibj.Timer;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;

public class SwerveHeadingController {
  public Rotation2d targetHeadingRadians;
  private double lastUpdatedTimestamp;

  public enum State {
    OFF,
    SNAP,
    STABILIZE
  }
  @AutoLogOutput(key = "Drive/HeadingController State")
  @Getter
  @Setter
  @Accessors(prefix = "m")
  private State mState = State.OFF;

  public void disable() {
    setState(State.OFF);
  }

  private SynchronousPIDF stabilizePID;
  private SynchronousPIDF snapPID;

  public void setSnapTarget(Rotation2d angle_rad) {
    targetHeadingRadians = angle_rad;
    setState(State.SNAP);
  }

  public void setStabilizeTarget(Rotation2d angle_rad) {
    targetHeadingRadians = angle_rad;
    setState(State.STABILIZE);
  }

  public Rotation2d getTargetHeading() {
    return targetHeadingRadians;
  }

  public SwerveHeadingController(SynchronousPIDF stabilizePID, SynchronousPIDF snapPID) {
    this.stabilizePID = stabilizePID;
    this.snapPID = snapPID;

    stabilizePID.setInputRange(-Math.PI, Math.PI);
    stabilizePID.setContinuous();

    stabilizePID.setOutputRange(-10 * Math.PI, 10 * Math.PI);

    snapPID.setInputRange(-Math.PI, Math.PI);
    snapPID.setContinuous();

    snapPID.setOutputRange(-10 * Math.PI, 10 * Math.PI);
    targetHeadingRadians = Rotation2d.identity();
    lastUpdatedTimestamp = Timer.getFPGATimestamp();

  }

  public double update(Rotation2d heading, double timestamp) {
    double correction = 0;
    double error = heading.minus(targetHeadingRadians).getRadians();
    double dt = timestamp - lastUpdatedTimestamp;
    switch (mState) {
      case OFF:
        break;
      case STABILIZE:
        correction = stabilizePID.calculate(error, dt);
        break;
      case SNAP:
        correction = snapPID.calculate(error, dt);
        break;
    }

    lastUpdatedTimestamp = timestamp;
    return correction;
  }
}
