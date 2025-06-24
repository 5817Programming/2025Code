package com.team5817.frc2025.subsystems.GamePieceVision;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Translation2d;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;

import org.littletonrobotics.junction.Logger;

import com.team5817.lib.drivers.Subsystem;
import com.team5817.lib.drivers.GamepieceVision.GamepieceVisionIO;
import com.team5817.lib.drivers.GamepieceVision.GamepieceVisionIO.GamepieceVisionIOData;
import com.team5817.lib.drivers.GamepieceVision.GamepieceVisionIOInputsAutoLogged;

public class GamepieceVision extends Subsystem {
  private final GamepieceVisionIO[] io;
  private final GamepieceVisionIOInputsAutoLogged[] inputs;

  private final GamepiecePoseAcceptor acceptor;
  private final RobotPoseSupplier robotPoseSupplier;

  private final List<TrackedGamepiece> recentObservations = new ArrayList<>();

  private static final double MAX_OBSERVATION_AGE_SEC = 0.5;
  private static final double FIELD_LENGTH = 16.5; // meters
  private static final double FIELD_WIDTH = 8.2;   // meters

  public GamepieceVision(GamepiecePoseAcceptor acceptor, RobotPoseSupplier robotPoseSupplier, GamepieceVisionIO... io) {
    this.acceptor = acceptor;
    this.robotPoseSupplier = robotPoseSupplier;
    this.io = io;
    this.inputs = new GamepieceVisionIOInputsAutoLogged[io.length];

    for (int i = 0; i < io.length; i++) {
      inputs[i] = new GamepieceVisionIOInputsAutoLogged();
    }
  }

  @Override
  public void readPeriodicInputs() {
    final double now = Timer.getFPGATimestamp();
    final Pose2d robotPose = robotPoseSupplier.getPose();
    final Rotation3d robotRotation = new Rotation3d(0, 0, robotPose.getRotation().getRadians());

    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      if(inputs[i].data.robotToTarget()==null){//If there is no result stop proccessing but process inputs needs a real value
        inputs[i].data = new GamepieceVisionIOData(new Translation3d(), inputs[i].data.timestamp());
        Logger.processInputs("GamepieceVision/Camera" + i, inputs[i]);
        continue;}
      Logger.processInputs("GamepieceVision/Camera" + i, inputs[i]);
      
      final Translation3d robotToTarget = inputs[i].data.robotToTarget();
      final Translation3d fieldGamepiecePose = new Translation3d(robotPose.getTranslation().wpi())

          .plus(robotToTarget.rotateBy(robotRotation));

      final double timestamp = inputs[i].data.timestamp();
      final Translation2d gamepiecePose2d = new Translation2d(fieldGamepiecePose.toTranslation2d());

      if (isValidGamepiecePose(fieldGamepiecePose)) {
        recentObservations.add(new TrackedGamepiece(gamepiecePose2d, timestamp));
      }
    }

    // Remove stale observations
    recentObservations.removeIf(obs -> (now - obs.timestamp > MAX_OBSERVATION_AGE_SEC));

    // Accept best observation
    final Optional<Translation2d> bestPose = getBestGamepiecePose();
    acceptor.accept(bestPose, now);

    // Log valid poses
    Logger.recordOutput("GamepieceVision/ValidObservations", recentObservations.stream()
        .map(obs -> obs.pose.wpi())
        .toArray(edu.wpi.first.math.geometry.Translation2d[]::new));
  }

  /** Returns the best gamepiece to track, currently the closest. */
  public Optional<Translation2d> getBestGamepiecePose() {
    try{
      return recentObservations.stream()
        .min(Comparator.comparingDouble(obs -> obs.pose.getTranslation().norm()))
        .map(obs -> obs.pose);
    }catch(Exception e){
      return Optional.empty();
    }
  }

  private boolean isValidGamepiecePose(Translation3d pose) {
    return pose.getX() >= 0 && pose.getX() <= FIELD_LENGTH
        && pose.getY() >= 0 && pose.getY() <= FIELD_WIDTH
        && Math.abs(pose.getZ()) < 0.5;
  }

  @Override
  public void outputTelemetry() {
    getBestGamepiecePose().ifPresent(best ->
        Logger.recordOutput("GamepieceVision/BestGamepiecePose", new edu.wpi.first.math.geometry.Pose2d(best.x(), best.y(), Rotation2d.kZero)));
  }

  private static class TrackedGamepiece {
    public final Translation2d pose;
    public final double timestamp;

    public TrackedGamepiece(Translation2d pose, double timestamp) {
      this.pose = pose;
      this.timestamp = timestamp;
    }
  }

  @FunctionalInterface
  public interface GamepiecePoseAcceptor {
    /**
     * Accepts a gamepiece pose observation.
     *
     * @param gamepiecePoseMeters The 2D pose of the gamepiece relative to the field.
     * @param timestampSeconds Timestamp of when the observation was captured.
     */
    void accept(Optional<Translation2d> gamepiecePoseMeters, double timestampSeconds);
  }

  @FunctionalInterface
  public interface RobotPoseSupplier {
    Pose2d getPose();
  }
}
