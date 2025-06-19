package com.team5817.frc2025;

import com.team5817.frc2025.generated.TunerConstants;
import com.team5817.frc2025.subsystems.Superstructure;
import com.team5817.frc2025.subsystems.Drive.Drive;
import com.team5817.frc2025.subsystems.Elevator.Elevator;
import com.team5817.frc2025.subsystems.Elevator.ElevatorConstants;
import com.team5817.frc2025.subsystems.EndEffector.EndEffectorConstants;
import com.team5817.frc2025.subsystems.EndEffector.EndEffectorConstants.EndEffectorWristConstants;
import com.team5817.frc2025.subsystems.EndEffector.EndEffectorWrist;
import com.team5817.frc2025.subsystems.Intake.Intake;
import com.team5817.frc2025.subsystems.Intake.IntakeConstants;
import com.team5817.frc2025.subsystems.Rollers.RollerSubsystem;
import com.team5817.frc2025.subsystems.Rollers.RollerSubsystemIO;
import com.team5817.frc2025.subsystems.Rollers.RollerSubsystemIOSim;
import com.team5817.frc2025.subsystems.Rollers.RollerSubsystemIOTalonFX;
import com.team5817.frc2025.subsystems.Vision.Vision;
import com.team5817.frc2025.subsystems.Vision.VisionConstants;
import com.team5817.frc2025.subsystems.Vision.VisionIO;
import com.team5817.frc2025.subsystems.Vision.VisionIOLimelight;
import com.team5817.frc2025.subsystems.Vision.VisionIOPhotonVisionSim;
import com.team5817.lib.RobotMode;
import com.team5817.lib.drivers.ServoMotorIO;
import com.team5817.lib.drivers.ServoMotorIOSim;
import com.team5817.lib.drivers.ServoMotorIOTalonFX;
import com.team5817.lib.swerve.GyroIO;
import com.team5817.lib.swerve.GyroIOPigeon2;
import com.team5817.lib.swerve.ModuleIO;
import com.team5817.lib.swerve.ModuleIOSim;
import com.team5817.lib.swerve.ModuleIOTalonFX;

import edu.wpi.first.math.system.plant.DCMotor;

public class RobotContainer {
  public Drive mDrive;
  public Elevator mElevator;
  public EndEffectorWrist mEndEffectorWrist;
  public RollerSubsystem<EndEffectorConstants.RollerState> mEndEffectorRollers;
  public Intake mIntake;
  public Vision mVision;
  public Superstructure mSuperstructure;

  public RobotContainer() {
    switch (RobotMode.mode) {
      case REAL:
        makeRealRobot();
        break;
      case REPLAY:
        makeEmptyRobot();
        break;
      case SIM:
      makeSimulatedRobot();
        break;
      default:
      makeEmptyRobot();
        break;
      
    }

    SubsystemManager mSubsystemManager = SubsystemManager.getInstance();

    mSuperstructure = new Superstructure(
        mDrive,
        mElevator,
        mEndEffectorWrist,
        mEndEffectorRollers,
        mIntake);

    mSubsystemManager.setSubsystems(
        mDrive,
        mSuperstructure,
        mVision,
        mElevator,
        mEndEffectorRollers,
        mEndEffectorWrist,
        mIntake);

  }

  public void makeRealRobot() {
    mEndEffectorWrist = new EndEffectorWrist(
        new ServoMotorIOTalonFX(EndEffectorWristConstants.kWristServoConstants));

    mEndEffectorRollers = new RollerSubsystem<EndEffectorConstants.RollerState>(
        EndEffectorConstants.RollerState.IDLE,
        "EndEffectorRollers",
        new RollerSubsystemIOTalonFX(
            Ports.ENDEFFECTOR_ROLLER,
            EndEffectorConstants.rollerConstants,
            1));

    mIntake = new Intake(
        new RollerSubsystemIOTalonFX(Ports.INTAKE_ROLLER, IntakeConstants.RollerConstants.motorConstants, 1),
        new RollerSubsystemIOTalonFX(Ports.SIDE_INDEXER, IntakeConstants.RollerConstants.motorConstants, 1),
        new RollerSubsystemIOTalonFX(Ports.BOTTOM_INDEXER, IntakeConstants.RollerConstants.motorConstants, 1),
        new ServoMotorIOTalonFX(ElevatorConstants.kElevatorServoConstants));
    mElevator = new Elevator(
        new ServoMotorIOTalonFX(ElevatorConstants.kElevatorServoConstants));
    mVision = new Vision(
      mDrive::addVisionMeasurement, 
      new VisionIOLimelight("limelight-up", mDrive::getHeading),
      new VisionIOLimelight("limelight-left", mDrive::getHeading),
      new VisionIOLimelight("limelight-right", mDrive::getHeading)
    );
    
    mDrive = new Drive(
      new GyroIOPigeon2(),
      new ModuleIOTalonFX(TunerConstants.FrontLeft),
      new ModuleIOTalonFX(TunerConstants.FrontRight),
      new ModuleIOTalonFX(TunerConstants.BackLeft),
      new ModuleIOTalonFX(TunerConstants.BackRight)
    );

  }

  public void makeSimulatedRobot() {
    mEndEffectorWrist = new EndEffectorWrist(
        new ServoMotorIOSim(EndEffectorConstants.EndEffectorWristConstants.kWristServoConstants));
    mEndEffectorRollers = new RollerSubsystem<EndEffectorConstants.RollerState>(
        EndEffectorConstants.RollerState.IDLE,
        "EndEffectorRollers",
        new RollerSubsystemIOSim(DCMotor.getKrakenX60(1), 1, .01));

    mIntake = new Intake(
        new RollerSubsystemIOSim(DCMotor.getKrakenX60(1), 1, 0.01),
        new RollerSubsystemIOSim(DCMotor.getKrakenX60(1), 1, 0.01),
        new RollerSubsystemIOSim(DCMotor.getKrakenX60(1), 1, 0.01),
        new ServoMotorIOSim(IntakeConstants.DeployConstants.kDeployServoConstants));
    mElevator = new Elevator(
        new ServoMotorIOSim(ElevatorConstants.kElevatorServoConstants));
   
    mDrive = new Drive(
      new GyroIO() {},
      new ModuleIOSim(TunerConstants.FrontLeft),
      new ModuleIOSim(TunerConstants.FrontRight),
      new ModuleIOSim(TunerConstants.BackLeft),
      new ModuleIOSim(TunerConstants.BackRight)
    );

    mVision = new Vision(
      mDrive::addVisionMeasurement, 
      new VisionIOPhotonVisionSim("limelight-up",VisionConstants.robotToCameraUp,mDrive::getPose),
      new VisionIOPhotonVisionSim("limelight-right",VisionConstants.robotToCameraLeft,mDrive::getPose),
      new VisionIOPhotonVisionSim("limelight-left",VisionConstants.robotToCameraRight,mDrive::getPose)
    );
  }
  public void makeEmptyRobot(){
    mEndEffectorWrist = new EndEffectorWrist(
        new ServoMotorIO(){});
    mEndEffectorRollers = new RollerSubsystem<EndEffectorConstants.RollerState>(
        EndEffectorConstants.RollerState.IDLE,
        "EndEffectorRollers",
        new RollerSubsystemIO(){});

    mIntake = new Intake(
        new RollerSubsystemIO(){},
        new RollerSubsystemIO(){},
        new RollerSubsystemIO(){},
        new ServoMotorIO(){});
    mElevator = new Elevator(new ServoMotorIO(){});
    mDrive = new Drive(
      new GyroIO() {},
      new ModuleIO() {},
      new ModuleIO() {},
      new ModuleIO() {},
      new ModuleIO() {}
    );

    mVision = new Vision(
      mDrive::addVisionMeasurement, 
      new VisionIO() {},
      new VisionIO() {},
      new VisionIO() {}
    );
  }
}
