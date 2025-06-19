package com.team5817.frc2025.subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import com.team5817.frc2025.loops.ILooper;
import com.team5817.frc2025.subsystems.Rollers.RollerSubsystemIO;
import com.team5817.lib.drivers.ServoMotorIO;
import com.team5817.lib.drivers.Subsystem;
import com.team5817.lib.requests.LambdaRequest;
import com.team5817.lib.requests.ParallelRequest;
import com.team5817.lib.requests.Request;

public class Intake extends Subsystem {

  private static IntakeRollers mIntakeRollers;
  private static IntakeDeploy mIntakeDeploy;

  private State mState = State.IDLE;

  public Intake(
      RollerSubsystemIO FeederIO,
      RollerSubsystemIO LowIndexerIO,
      RollerSubsystemIO SideIndexerIO,
      ServoMotorIO DeployIO) {
    mIntakeRollers = new IntakeRollers(FeederIO, LowIndexerIO, SideIndexerIO);
    mIntakeDeploy = new IntakeDeploy(DeployIO.getConstants(), DeployIO);
  }

  public enum State {
    IDLE(IntakeRollers.State.IDLE, IntakeDeploy.State.DISABLE),
    INTAKING(IntakeRollers.State.INTAKING, IntakeDeploy.State.GROUND),
    HALF_INTAKING(IntakeRollers.State.HALF_INTAKING, IntakeDeploy.State.GROUND),
    EXHAUSTING(IntakeRollers.State.EXHAUST, IntakeDeploy.State.GROUND),
    IDLE_EXAUST(IntakeRollers.State.IDLE_EXAUST, IntakeDeploy.State.DISABLE),
    HUMAN(IntakeRollers.State.INTAKING, IntakeDeploy.State.HUMAN),
    STOW(IntakeRollers.State.IDLE, IntakeDeploy.State.STOW);

    final IntakeRollers.State rollerState;
    final IntakeDeploy.State deployState;

    State(IntakeRollers.State rollerState, IntakeDeploy.State deployState) {
      this.rollerState = rollerState;
      this.deployState = deployState;
    }
  }

  @Override
  public void readPeriodicInputs() {
    mIntakeRollers.readPeriodicInputs();
    mIntakeDeploy.readPeriodicInputs();
  }

  @Override
  public void writePeriodicOutputs() {
    mIntakeRollers.writePeriodicOutputs();
    mIntakeDeploy.writePeriodicOutputs();
  }

  @Override
  public void stop() {
    mIntakeRollers.stop();
    mIntakeDeploy.stop();
  }

  @Override
  public boolean checkDeviceConfiguration() {
    return mIntakeRollers.checkDeviceConfiguration() && mIntakeDeploy.checkDeviceConfiguration();
  }

  @Override
  public boolean checkSystem() {
    return mIntakeRollers.checkSystem() && mIntakeDeploy.checkSystem();
  }

  @Override
  public void registerEnabledLoops(ILooper enabledLooper) {
    mIntakeRollers.registerEnabledLoops(enabledLooper);
    mIntakeDeploy.registerEnabledLoops(enabledLooper);
  }

  @Override
  public void outputTelemetry() {
    Logger.recordOutput("Intake/State", mState);
    Logger.recordOutput("Intake/AtState", stateRequest(mState).isFinished());
  }

  public void conformToState(State state) {
    stateRequest(state).act();
  }

  public Request stateRequest(State state) {
    return new ParallelRequest(
        new LambdaRequest(() -> this.mState = state),
        mIntakeRollers.stateRequest(state.rollerState),
        mIntakeDeploy.stateRequest(state.deployState));
  }
}
