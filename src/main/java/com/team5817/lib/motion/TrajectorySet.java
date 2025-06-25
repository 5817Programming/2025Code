package com.team5817.lib.motion;

import java.util.ArrayList;
import java.util.List;

import com.team254.lib.geometry.Pose2d;

/**
 * A set of trajectories that can be mirrored.
 */
public class TrajectorySet {
    List<Trajectory> mTrajectorySet = new ArrayList<>();
    boolean mirrored;

    /**
     * Constructs a TrajectorySet with the given trajectories.
     * @param trajectories The trajectories to add to the set.
     */
    public TrajectorySet(Trajectory... trajectories) {
        mirrored = false;
        for (Trajectory t : trajectories) {
            t.setMirrored(mirrored);
            mTrajectorySet.add(t);
        }
    }

    /**
     * Constructs a TrajectorySet with the given trajectories and mirrored state.
     * @param mirrored Whether the trajectories should be mirrored.
     * @param trajectories The trajectories to add to the set.
     */
    public TrajectorySet(Boolean mirrored, Trajectory... trajectories) {
       int count = 0; 
        this.mirrored = mirrored;
        for (Trajectory t : trajectories) {
            count++;
            if(t == null){
                System.out.println("Trajectory"+count+" is null");
                continue;
            }
            t.setMirrored(mirrored);
            mTrajectorySet.add(t);
        }

    }

    /**
     * Returns the next trajectory in the set and removes it from the set.
     * @return The next trajectory.
     */
    public Trajectory next() {
        try{
            return mTrajectorySet.remove(0);
        }catch(Exception e){
            System.out.println("Cannot find next trajectory");
            throw(e);
        }
    }

    /**
     * Returns the first trajectory in the set without removing it.
     * @return The first trajectory.
     */
    public Trajectory first() {
        return mTrajectorySet.get(0);
    }

    /**
     * Returns the initial pose of the first trajectory in the set.
     * @return The initial pose.
     */
    public Pose2d initalPose() {
        return mTrajectorySet.get(0).get().getCurrentState().getPose();
    }

}