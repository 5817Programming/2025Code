package com.team5817.frc2025.subsystems.Intake;

import com.team5817.frc2025.subsystems.Intake.IntakeConstants.RollerConstants.FeederState;
import com.team5817.frc2025.subsystems.Intake.IntakeConstants.RollerConstants.LowIndexerState;
import com.team5817.frc2025.subsystems.Intake.IntakeConstants.RollerConstants.SideIndexerState;
import com.team5817.frc2025.subsystems.Rollers.RollerSubsystem;
import com.team5817.frc2025.subsystems.Rollers.RollerSubsystemIO;
import com.team5817.lib.drivers.Subsystem;
import com.team5817.lib.requests.ParallelRequest;
import com.team5817.lib.requests.Request;

import lombok.Getter;

public class IntakeRollers extends Subsystem {

  private final RollerSubsystem<FeederState> feeder;
  private final RollerSubsystem<LowIndexerState> lowIndexer;
  private final RollerSubsystem<SideIndexerState> sideIndexer;

  public IntakeRollers(
      RollerSubsystemIO FeederIO,
      RollerSubsystemIO LowIndexerIO,
      RollerSubsystemIO SideIndexerIO) {
    this.feeder = new RollerSubsystem<FeederState>(FeederState.IDLE, "Intake Feeder", FeederIO);
    this.lowIndexer = new RollerSubsystem<LowIndexerState>(LowIndexerState.IDLE, "Intake Bottom Indexer", LowIndexerIO);
    this.sideIndexer = new RollerSubsystem<SideIndexerState>(SideIndexerState.IDLE, "Intake Side Indexer",
        SideIndexerIO);
  }

  public enum State {
    IDLE(FeederState.IDLE, LowIndexerState.IDLE, SideIndexerState.IDLE),
    INTAKING(FeederState.INTAKING, LowIndexerState.INTAKING, SideIndexerState.INTAKING),
    HALF_INTAKING(FeederState.INTAKING, LowIndexerState.IDLE, SideIndexerState.IDLE),
    EXHAUST(FeederState.EXHAUST, LowIndexerState.EXHAUST, SideIndexerState.EXHAUST),
    IDLE_EXAUST(FeederState.IDLE, LowIndexerState.SLOW_EXAUST, SideIndexerState.SLOW_EXAUST);

    @Getter
    private final FeederState feederState;
    @Getter
    private final LowIndexerState lowIndexerState;
    @Getter
    private final SideIndexerState sideIndexerState;

    State(FeederState feederState, LowIndexerState lowIndexerState, SideIndexerState sideIndexerState) {
      this.feederState = feederState;
      this.lowIndexerState = lowIndexerState;
      this.sideIndexerState = sideIndexerState;
    }
  }

  @Override
  public void readPeriodicInputs() {
    feeder.readPeriodicInputs();
    lowIndexer.readPeriodicInputs();
    sideIndexer.readPeriodicInputs();
  }

  @Override
  public void writePeriodicOutputs() {
    feeder.writePeriodicOutputs();
    lowIndexer.writePeriodicOutputs();
    sideIndexer.writePeriodicOutputs();
  }

  @Override
  public boolean checkSystem() {
    return feeder.allOK() && lowIndexer.allOK() && sideIndexer.allOK();
  }

  public Request stateRequest(State state) {
    return new ParallelRequest(feeder.stateRequest(state.feederState), sideIndexer.stateRequest(state.sideIndexerState),
        lowIndexer.stateRequest(state.lowIndexerState));
  }
}
