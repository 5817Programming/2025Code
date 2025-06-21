package com.team5817.lib.diagnostic;

import com.team5817.frc2025.autos.AutoBase;
import com.team5817.lib.diagnostic.FeedForwardCharacterization.FeedForwardCharacterizationData;
import com.team5817.lib.drivers.Servos.ServoMotorSubsystem;

/**
 * Characterize class extends AutoBase to perform feedforward characterization on a subsystem.
 */
public class Characterize extends AutoBase {

    FeedForwardCharacterization mCharacterization;

    /**
     * Constructs a Characterize object.
     *
     * @param name the name of the characterization
     * @param subsystem the subsystem to be characterized
     * @param forward the direction of the characterization
     */
    public Characterize(String name, ServoMotorSubsystem subsystem, boolean forward) {
        mCharacterization = new FeedForwardCharacterization(subsystem, forward, new FeedForwardCharacterizationData(name));
    }

    /**
     * Executes the characterization routine.
     */
    public void routine() {
        r(mCharacterization);
    }
    
}
