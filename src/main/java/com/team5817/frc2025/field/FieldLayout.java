package com.team5817.frc2025.field;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team5817.frc2025.RobotConstants;
import com.team5817.frc2025.field.AlignmentPoint.AlignmentType;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.io.IOException;
import java.util.HashMap;

/**
 * Contains various field dimensions and useful reference points. Dimensions are
 * in meters, and sets
 * of corners start in the lower left moving clockwise. <b>All units in
 * Meters</b> <br>
 * <br>
 *
 * <p>
 * All translations and poses are stored with the origin at the rightmost point
 * on the BLUE
 * ALLIANCE wall.<br>
 * <br>
 * Length refers to the <i>x</i> direction (as described by wpilib) <br>
 * Width refers to the <i>y</i> direction (as described by wpilib)
 */
public class FieldLayout {
	public static double kFieldLength = 17.55; 
	public static double kFieldWidth = 8.05; 

	public static final AprilTagFieldLayout kTagMap;
	static {
		try {
			kTagMap = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeWelded.m_resourceFile); 
		} catch (IOException e) {
			throw new RuntimeException(e);
		}
	}
     
        private static final AlignmentPoint kReefToCenterAlign = new AlignmentPoint(new Translation2d(-RobotConstants.kBumberSideLength/2, 0.0),AlignmentType.ALGAE_CLEAN);
        private static final AlignmentPoint kReefToRightAlign = new AlignmentPoint(new Translation2d(-(RobotConstants.kBumberSideLength+0.3358)/2, -0.1643),AlignmentType.CORAL_SCORE,AlignmentType.CORAL_SCORE_RIGHT);
        private static final AlignmentPoint kReefToLeftAlign = new AlignmentPoint(new Translation2d(-(RobotConstants.kBumberSideLength+0.3358)/2, 0.1643),AlignmentType.CORAL_SCORE,AlignmentType.CORAL_SCORE_LEFT);

        private static final AlignmentPoint kHumanToCenterAlign = new AlignmentPoint(new Translation2d(RobotConstants.kBumberSideLength/2, 0.0),AlignmentType.HUMAN);
        private static final AlignmentPoint kHumanToRightAlign = new AlignmentPoint(new Translation2d(RobotConstants.kBumberSideLength/2, -0.55),AlignmentType.HUMAN);
        private static final AlignmentPoint kHumanToLeftAlign = new AlignmentPoint(new Translation2d(RobotConstants.kBumberSideLength/2, 0.55),AlignmentType.HUMAN);

        private static final AlignmentPoint kProcessorToCenterAlign = new AlignmentPoint(new Translation2d(-RobotConstants.kBumberSideLength/2, 0.0),AlignmentType.ALGAE_SCORE);
    public static class Red {

        public static final HashMap<Integer, AprilTag> kAprilTagMap = new HashMap<>();
        // From field manual: 46.355 cm
        // From CAD: 46.2534 cm
        // From step layout marking: 46.2788 cm
        // Old value: 46.272  cm



      
        public static final AprilTag kAprilTag6 = new AprilTag(
                6,
                true,
                kReefToCenterAlign,
                kReefToLeftAlign,
                kReefToRightAlign
        );
        public static final AprilTag kAprilTag7 = new AprilTag(
                7,
                true,
                kReefToCenterAlign,
                kReefToLeftAlign,
                kReefToRightAlign
        );
        public static final AprilTag kAprilTag8 = new AprilTag(
                8,
                true,
                kReefToCenterAlign,
                kReefToLeftAlign,
                kReefToRightAlign
        );
        public static final AprilTag kAprilTag9 = new AprilTag(
                9,
                true,
                kReefToCenterAlign,
                kReefToLeftAlign,
                kReefToRightAlign
        );
        public static final AprilTag kAprilTag10 = new AprilTag(
                10,
                true,
                kReefToCenterAlign,
                kReefToLeftAlign,
                kReefToRightAlign
        );
        public static final AprilTag kAprilTag11 = new AprilTag(
                11,
                true,
                kReefToCenterAlign,
                kReefToLeftAlign,
                kReefToRightAlign
        );
        public static final AprilTag kAprilTag1 = new AprilTag(
                1,
                false,
                kHumanToCenterAlign,
                kHumanToLeftAlign,
                kHumanToRightAlign
        );
        public static final AprilTag kAprilTag2 = new AprilTag(
                2,
                false,
                kHumanToCenterAlign,
                kHumanToLeftAlign,
                kHumanToRightAlign
        );
        public static final AprilTag kAprilTag3 = new AprilTag(
                3,
                true,
                kProcessorToCenterAlign,
                kProcessorToCenterAlign,
                kProcessorToCenterAlign
        );
       


        static {
                kAprilTagMap.put(6, kAprilTag6);
                kAprilTagMap.put(7, kAprilTag7);
                kAprilTagMap.put(8, kAprilTag8);
                kAprilTagMap.put(9, kAprilTag9);
                kAprilTagMap.put(10, kAprilTag10);
                kAprilTagMap.put(11, kAprilTag11);
                kAprilTagMap.put(1, kAprilTag1);
                kAprilTagMap.put(2, kAprilTag2);
                kAprilTagMap.put(3, kAprilTag3);
        }


    }

    public static class Blue {
        public static final HashMap<Integer, AprilTag> kAprilTagMap = new HashMap<>();
        // From field manual: 46.355 cm
        // From CAD: 46.2534 cm
        // From step layout marking: 46.2788 cm
        // Old value: 46.272  cm
      
        public static final AprilTag kAprilTag17 = new AprilTag(
                17,
                true,
                kReefToCenterAlign,
                kReefToLeftAlign,
                kReefToRightAlign
        );
        public static final AprilTag kAprilTag18 = new AprilTag(
                18,
                true,
                kReefToCenterAlign,
                kReefToLeftAlign,
                kReefToRightAlign
        );
        public static final AprilTag kAprilTag19 = new AprilTag(
                19,
                true,
                kReefToCenterAlign,
                kReefToLeftAlign,
                kReefToRightAlign
        );
        public static final AprilTag kAprilTag20 = new AprilTag(
                20,
                true,
                kReefToCenterAlign,
                kReefToLeftAlign,
                kReefToRightAlign
        );
        public static final AprilTag kAprilTag21 = new AprilTag(
                21,
                true,
                kReefToCenterAlign,
                kReefToLeftAlign,
                kReefToRightAlign
        );
        public static final AprilTag kAprilTag22 = new AprilTag(
                22,
                true,
                kReefToCenterAlign,
                kReefToLeftAlign,
                kReefToRightAlign
        );
        public static final AprilTag kAprilTag12 = new AprilTag(
                12,
                false,
                kHumanToCenterAlign,
                kHumanToLeftAlign,
                kHumanToRightAlign
        );
        public static final AprilTag kAprilTag13 = new AprilTag(
                13,
                false,
                kHumanToCenterAlign,
                kHumanToLeftAlign,
                kHumanToRightAlign
        );
        public static final AprilTag kAprilTag16 = new AprilTag(
                16,
                true,
                kProcessorToCenterAlign,
                kProcessorToCenterAlign,
                kProcessorToCenterAlign
        );
       


        static {
                kAprilTagMap.put(17, kAprilTag17);
                kAprilTagMap.put(18, kAprilTag18);
                kAprilTagMap.put(19, kAprilTag19);
                kAprilTagMap.put(20, kAprilTag20);
                kAprilTagMap.put(21, kAprilTag21);
                kAprilTagMap.put(22, kAprilTag22);
                kAprilTagMap.put(12, kAprilTag12);
                kAprilTagMap.put(13, kAprilTag13);
                kAprilTagMap.put(16, kAprilTag16);
        }


    }
	
	/**
     * Handles the alliance flip for a given pose.
     *
     * @param blue_pose The pose in the blue alliance frame.
     * @param is_red_alliance Whether the alliance is red.
     * @return The pose in the correct alliance frame.
     */
	public static Pose2d handleAllianceFlip(Pose2d blue_pose, boolean is_red_alliance) {
		if (is_red_alliance) {
			blue_pose = blue_pose.mirrorAboutX(kFieldLength / 2.0);
		}
		return blue_pose;
	}

	/**
     * Handles the alliance flip for a given translation.
     *
     * @param blue_translation The translation in the blue alliance frame.
     * @param is_red_alliance Whether the alliance is red.
     * @return The translation in the correct alliance frame.
     */
	public static Translation2d handleAllianceFlip(Translation2d blue_translation, boolean is_red_alliance) {
		if (is_red_alliance) {
			blue_translation = blue_translation.mirrorAboutX(kFieldLength / 2.0);
		}
		return blue_translation;
	}

	/**
     * Handles the alliance flip for a given rotation.
     *
     * @param blue_rotation The rotation in the blue alliance frame.
     * @param is_red_alliance Whether the alliance is red.
     * @return The rotation in the correct alliance frame.
     */
	public static Rotation2d handleAllianceFlip(Rotation2d blue_rotation, boolean is_red_alliance) {
		if (is_red_alliance) {
			blue_rotation = blue_rotation.mirrorAboutX();
		}
		return blue_rotation;
	}

	/**
     * Calculates the distance from the alliance wall.
     *
     * @param x_coordinate The x-coordinate.
     * @param is_red_alliance Whether the alliance is red.
     * @return The distance from the alliance wall.
     */
	public static double distanceFromAllianceWall(double x_coordinate, boolean is_red_alliance) {
		if (is_red_alliance) {
			return kFieldLength - x_coordinate;
		}
		return x_coordinate;
	}

	/**
     * Gets the reef pose based on the alliance.
     *
     * @return The reef pose in the correct alliance frame.
     */
    public static Translation2d getReefPose() {
        var blue = new Translation2d(4.5,4);
        if(DriverStation.getAlliance().get().equals(Alliance.Red))
                return blue.mirrorAboutX(kFieldLength/2);
        else  
                return blue;
    }
}
