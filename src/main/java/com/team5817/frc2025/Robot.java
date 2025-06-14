// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5817.frc2025;

import java.util.Optional;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.swerve.ChassisSpeeds;
import com.team5817.BuildConstants;
import com.team5817.frc2025.autos.AutoBase;
import com.team5817.frc2025.autos.AutoExecuter;
import com.team5817.frc2025.autos.AutoModeSelector;
import com.team5817.frc2025.autos.Modes.Characterize;
import com.team5817.frc2025.autos.TrajectoryLibrary.l;
import com.team5817.frc2025.controlboard.ControlBoard;
import com.team5817.frc2025.controlboard.DriverControls;
import com.team5817.frc2025.loops.Looper;
import com.team5817.frc2025.subsystems.Superstructure;
import com.team5817.frc2025.subsystems.Drive.Drive;
import com.team5817.frc2025.subsystems.Elevator.Elevator;
import com.team5817.frc2025.subsystems.EndEffector.EndEffectorRollers;
import com.team5817.frc2025.subsystems.EndEffector.EndEffectorWrist;
import com.team5817.frc2025.subsystems.Intake.Intake;
import com.team5817.frc2025.subsystems.vision.VisionDeviceManager;
import com.team5817.lib.Elastic;
import com.team5817.lib.Util;
import com.team5817.lib.RobotMode;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

/**
 * The main robot class that extends LoggedRobot and contains the robot's
 * lifecycle methods.
 */
public class Robot extends LoggedRobot {
  SubsystemManager mSubsystemManager;
  private AutoExecuter mAutoExecuter;
  private AutoModeSelector mAutoModeSelector = new AutoModeSelector();
  DriverControls controls;
  ControlBoard controlBoard = ControlBoard.getInstance();
  private final Looper mEnabledLooper = new Looper();

  SwerveDriveSimulation mDriveSim;
  Drive mDrive;

  @SuppressWarnings("resource")
  /**
   * This method is called when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    if (Robot.isReal())
      RobotMode.setMode(RobotMode.Mode.REAL);

    DriverStation.silenceJoystickConnectionWarning(true);
    for (int port = 5800; port <= 5809; port++) {
      PortForwarder.add(port, "limelight-right.local", port);
      PortForwarder.add(port + 10, "limelight-left.local", port);
      PortForwarder.add(port + 20, "limelight-up.local", port);
    }

    DriverStation.startDataLog(DataLogManager.getLog());

    RobotState.getInstance().reset();

    Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);

    if (RobotMode.isReal()) {
      Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    } else {
      if (RobotMode.isReplay()) {
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new
                                                                                              // log
        setUseTiming(false);
      } else {
        mDriveSim = new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(13, 3, Rotation2d.identity()).wpi());
        Drive.registerDriveSimulation(mDriveSim);
        SimulatedArena.getInstance().addDriveTrainSimulation(mDriveSim);
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
      }
    }

    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
                    // be added.
    l.init();

    mDrive = Drive.getInstance();
    mSubsystemManager = SubsystemManager.getInstance();

    Elastic.selectTab("Pre Match");

    controls = new DriverControls();
    mSubsystemManager.setSubsystems(
        Drive.getInstance(),
        Superstructure.getInstance(),
        VisionDeviceManager.getInstance(),
        Elevator.getInstance(),
        EndEffectorRollers.getInstance(),
        EndEffectorWrist.getInstance(),
        Intake.getInstance());

    mSubsystemManager.registerEnabledLoops(mEnabledLooper);
    mEnabledLooper.start();
    Logger.recordOutput("isComp", RobotConstants.isComp);
  }

  /**
   * This method is called periodically, regardless of the robot's mode.
   */
  boolean needsZero = true;

  @Override
  public void robotPeriodic() {
    if (needsZero && DriverStation.getAlliance().isPresent()) {
      mDrive.zeroGyro(Util.isRed().get() ? 0 : 180);
      needsZero = false;
    }
    Logger.recordOutput("Elastic/Match Time", Timer.getMatchTime());
    mEnabledLooper.update();
    RobotVisualizer.outputTelemetry();
  }

  boolean disableGyroReset = false;

  /**
   * This method is called once each time the robot enters autonomous mode.
   */
  @Override
  public void autonomousInit() {
    neverEnabled = false;
    Elastic.selectTab("Autonomous");
    mAutoExecuter.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }

  boolean neverEnabled = true;

  /**
   * This method is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
    EndEffectorWrist.getInstance().setManualOffset(0);
    Elevator.getInstance().setManualOffset(0);
    neverEnabled = false;
    mDrive.setControlState(Drive.DriveControlState.OPEN_LOOP);

    Elastic.selectTab("Teleoperated");
    mDrive.resetModulesToAbsolute();
    mDrive.feedTeleopSetpoint(new ChassisSpeeds());
    mDrive.setOpenLoop(new ChassisSpeeds());
  }

  /**
   * This method is called periodically during teleoperated mode.
   */
  @Override
  public void teleopPeriodic() {
    controls.twoControllerMode();
    controlBoard.update();

    mDrive.feedTeleopSetpoint(ChassisSpeeds.fromFieldRelativeSpeeds(
        controlBoard.getSwerveTranslation().x(),
        controlBoard.getSwerveTranslation().y(),
        controlBoard.getSwerveRotation(),
        Util.robotToFieldRelative(mDrive.getHeading(), DriverStation.getAlliance().get().equals(Alliance.Red))));

  }

  /**
   * This method is called once each time the robot is disabled.
   */
  @Override
  public void disabledInit() {
    mSubsystemManager.stop();

    if (mAutoExecuter != null) {
      mAutoExecuter.stop();
    }
    mAutoExecuter = new AutoExecuter();
  }

  /**
   * This method is called periodically when the robot is disabled.
   */
  @Override
  public void disabledPeriodic() {
    l.update();
    // if(mVision.getMovingAverage().getSize()!=0&&neverEnabled)
    // mDrive.zeroGyro(mVision.getMovingAverage().getAverage());
    mAutoModeSelector.updateModeCreator();
    Optional<AutoBase> autoMode = mAutoModeSelector.getAutoMode();
    if (autoMode.isPresent() && (autoMode.get() != mAutoExecuter.getAuto())) {
      if (RobotMode.isSim())
        autoMode.get().registerDriveSimulation(mDriveSim);
      mAutoExecuter.setAuto(autoMode.get());
    }
  }

  /**
   * This method is called once each time the robot enters test mode.
   */
  @Override
  public void testInit() {
    Elastic.selectTab("Systems Test");

    mAutoExecuter.setAuto(new Characterize(Elevator.getInstance(), true));
    mAutoExecuter.start();
  }

  /**
   * This method is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    Elevator.getInstance().writePeriodicOutputs();
    Elevator.getInstance().outputTelemetry();
    // controls.testMode();
    // controlBoard.update();

    // mDrive.feedTeleopSetpoint(ChassisSpeeds.fromFieldRelativeSpeeds(
    // controlBoard.getSwerveTranslation().x(),
    // controlBoard.getSwerveTranslation().y(),
    // controlBoard.getSwerveRotation(),
    // Util.robotToFieldRelative(mDrive.getHeading(),
    // DriverStation.getAlliance().get().equals(Alliance.Red))));
  }

  /**
   * This method is called once when the simulation is initialized.
   */
  @Override
  public void simulationInit() {
    if (RobotMode.isSim())
      SimulatedArena.getInstance().resetFieldForAuto();
  }

  /**
   * This method is called periodically during simulation.
   */
  @Override
  public void simulationPeriodic() {
    if (RobotMode.isSim()) {
      SimulatedArena.getInstance().simulationPeriodic();
      Logger.recordOutput("FieldSimulation/RobotPosition", mDriveSim.getSimulatedDriveTrainPose());
      Logger.recordOutput(
          "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
      Logger.recordOutput(
          "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
    }
  }
}
