package com.team5817.frc2025.subsystems.Drive;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import com.team254.lib.geometry.Pose2d;
import com.team5817.frc2025.field.AlignmentPoint;
import com.team5817.frc2025.field.AprilTag;
import com.team5817.frc2025.field.FieldLayout;
import com.team5817.frc2025.field.AlignmentPoint.AlignmentType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AutoAlignPointSelector {
    public static AlignmentPoint a;
    /**
     * Gets the set of AprilTags based on the current alliance color.
     * 
     * @return A map of AprilTag IDs to AprilTag objects.
     */
    private static Map<Integer, AprilTag> getTagSet() {
        if(DriverStation.getAlliance().isEmpty())
            return new HashMap<Integer, AprilTag>();
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            return FieldLayout.Red.kAprilTagMap;
        } else {
            return FieldLayout.Blue.kAprilTagMap;
        }
    }

    /**
     * Finds the nearest AprilTag to a given point that supports the desired alignment.
     * 
     * @param tagMap The map of AprilTags.
     * @param point The current position.
     * @param desiredAlignment The desired alignment type.
     * @return An Optional containing the nearest AprilTag, or empty if none found.
     */
    private static Optional<AprilTag> getNearestTag(Map<Integer, AprilTag> tagMap, Pose2d point,
            AlignmentType desiredAlignment) {
        double closestDistance = Integer.MAX_VALUE;
        Optional<AprilTag> closestTag = Optional.empty();
        for (int i : tagMap.keySet()) {
            for (AlignmentPoint a : tagMap.get(i).getAllAlignmentPoints()) {
                if (a.getAllowedAllignments().contains(desiredAlignment)) {
                    double distance = tagMap.get(i).getFieldToTag().distance(point);
                    if (distance < closestDistance) {
                        closestDistance = distance;
                        closestTag = Optional.of(tagMap.get(i));
                    }
                    break;
                }
            }

        }
        return closestTag;
    }

    /**
     * Finds the Pose2d in a list that is closest to a given point.
     * 
     * @param from The starting position.
     * @param to The list of target positions.
     * @return The Pose2d that is closest to the starting position.
     */
    private static Pose2d minimizeDistance(Pose2d from, List<Pose2d> to, List<AlignmentPoint> points) {
        if (to.size() == 0) {
            return null;
        }
        double closestDistance = Integer.MAX_VALUE;
        Pose2d closestPose = to.get(0);
        for (int i = 0; i < to.size(); i++) {
            double distance = from.distance(to.get(i));
            if (distance < closestDistance) {
                closestDistance = distance;
                closestPose = to.get(i);
                a = points.get(i);
                Logger.recordOutput("AutoAlign/Closest Point", a.getAllowedAllignments().toString());
            }
        }
        return (closestPose);
    }

    /**
     * Gets the nearest alignment point on an AprilTag to a given point.
     * 
     * @param tag The AprilTag.
     * @param point The current position.
     * @param desiredAlignment The desired alignment type.
     * @return The nearest alignment point as a Pose2d.
     */
    private static Pose2d getNearestAlignment(AprilTag tag, Pose2d point, AlignmentType desiredAlignment) {
        List<Pose2d> poses = new ArrayList<>();
        List<AlignmentPoint> points = new ArrayList<>();
        for (AlignmentPoint a : tag.getAllAlignmentPoints()) {
            if (a.getAllowedAllignments().contains(desiredAlignment)){
                poses.add(tag.getFieldToTag().transformBy(Pose2d.fromTranslation(a.getTranslation())));
                points.add(a);
            }
        }
        return minimizeDistance(point, poses,points);
    }

    /**
     * Chooses the target alignment point based on the current position and desired alignment.
     * 
     * @param currentPoint The current position.
     * @param desiredAlignment The desired alignment type.
     * @return The target alignment point as a Pose2d.
     */
    public static Pose2d chooseTargetPoint(Pose2d currentPoint, AlignmentType desiredAlignment) {
        Map<Integer, AprilTag> mTagMap = getTagSet();
        Optional<AprilTag> closestTag = getNearestTag(mTagMap, currentPoint, desiredAlignment);
        Pose2d targetPose = getNearestAlignment(closestTag.get(), currentPoint, desiredAlignment);
        return targetPose;
    }
}
