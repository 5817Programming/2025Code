package com.team5817.frc2025.field;

import java.util.List;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Translation2d;

/**
 * Represents an alignment point on the field with a specific pose and allowed alignments.
 */
public class AlignmentPoint {

    private List<AlignmentType> mAllowedAllignments;
    private Translation2d mPose;

    /**
     * Constructs an AlignmentPoint with the specified pose and allowed alignments.
     *
     * @param pose the position of the alignment point
     * @param allowed_allignments the types of alignments allowed at this point
     */
    public AlignmentPoint (Translation2d pose, AlignmentType... allowed_allignments){
        this.mPose = pose;
        this.mAllowedAllignments = List.of(allowed_allignments);
    }

    /**
     * Enum representing the types of alignments that can be allowed at an alignment point.
     */
    public enum AlignmentType{
        CORAL_SCORE,
        CORAL_SCORE_LEFT,
        CORAL_SCORE_RIGHT,
        ALGAE_CLEAN,
        ALGAE_SCORE,
        HUMAN,
        NONE;
        public Pose2d tolerance;
        private AlignmentType(Pose2d tolerance){
            this.tolerance = tolerance;
        }
        private AlignmentType(){
            this.tolerance = new Pose2d(.02, 0.03, 3);
        }
    }

    /**
     * Gets the translation (position) of the alignment point.
     *
     * @return the translation of the alignment point
     */
    public Translation2d getTranslation(){
        return mPose;
    }

    /**
     * Gets the list of allowed alignments at the alignment point.
     *
     * @return the list of allowed alignments
     */
    public List<AlignmentType> getAllowedAllignments(){
        return mAllowedAllignments;
    }
    
}
