package com.team5817.frc2025.autos.Modes;

import com.team5817.frc2025.autos.AutoBase;
import com.team5817.lib.diagnostic.FeedForwardCharacterization;
import com.team5817.lib.diagnostic.FeedForwardCharacterization.FeedForwardCharacterizationData;
import com.team5817.lib.drivers.Servos.ServoMotorSubsystem;

public class Characterize extends AutoBase{

    FeedForwardCharacterization characterization;
    ServoMotorSubsystem subsystem;
    boolean forwards;
    public Characterize(ServoMotorSubsystem subsystem,boolean forwards){
        this.subsystem = subsystem;
        this.forwards = forwards;
    }
    @Override
    public void routine() {
        FeedForwardCharacterizationData data = new FeedForwardCharacterizationData(subsystem.getClass().getName());
        characterization = new FeedForwardCharacterization(subsystem, forwards, data);
        r(characterization);
    }
    
}
