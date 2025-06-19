package com.team5817.frc2025.autos;

import com.team5817.frc2025.autos.Actions.Action;
import edu.wpi.first.wpilibj.Timer;

/**
 * Abstract base class for autonomous routines.
 */
public abstract class AutoBase {

    protected double startTime;
    boolean mActive = false;

    /**
     * Starts the autonomous routine.
     */
    public void start() {
        mActive = true;
        startTime = Timer.getTimestamp();
        routine();
        end();
    }

    /**
     * Ends the autonomous routine and prints the duration.
     */
    public void end() {
        System.out.println("Auto Ended in " + (Timer.getTimestamp() - startTime) + " seconds");
    }

    /**
     * The routine to be implemented by subclasses.
     */
    public abstract void routine();

    /**
     * Stops the autonomous routine.
     */
    public void stop() {
        mActive = false;
    }

    /**
     * Runs a given action.
     * 
     * @param action The action to run.
     */
    public void r(Action action) {
        action.start();
        if (!mActive)
            throw new Error("Action ran while auto is inactive");
        while (mActive && !action.isFinished()) {
            action.update();
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        action.done();
        if (!mActive)
            throw new Error("Action Interrupted");
    }
}