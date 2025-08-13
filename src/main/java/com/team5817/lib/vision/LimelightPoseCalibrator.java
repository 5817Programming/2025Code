package com.team5817.lib.vision;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.InvalidPathException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.alibaba.fastjson2.JSON;
import com.alibaba.fastjson2.JSONException;
import com.alibaba.fastjson2.JSONReader.Feature;
import com.alibaba.fastjson2.annotation.JSONType;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;

public class LimelightPoseCalibrator {

    @JSONType(deserializeFeatures = Feature.FieldBased)
    public static class PoseCalibrationInfo {
        public String limelightName;
        // Assumes that the tag yaw, pitch, and roll are zero
        public double[] robotToTagTranslationInches;

        PoseCalibrationInfo() {
            robotToTagTranslationInches = new double[3];
        }

        public Pose3d getRobotPoseInTagSpaceMeters() {
            return new Pose3d(Units.inchesToMeters(robotToTagTranslationInches[0]), Units.inchesToMeters(robotToTagTranslationInches[1]), 
                    -Units.inchesToMeters(robotToTagTranslationInches[2]), new Rotation3d(0.0, 0.0, Units.degreesToRadians(180.0)));
        }
    }

    /**
     * The limelight defines tag space as the following:
     * +x points to the right of the target (if looking at the target)
     * +y point downward
     * +z points out of the tag
     * 
     * We want to convert this to something more similar to robot space:
     * +x points out of the tag
     * +y points to the left of the tag if you embody the tag (to the right if you lopok at the tag)
     * +z points upward
     */
    private static final Pose3d limelightTagSpaceToOurTagSpace(double[] tagSpacePose) {
        return new Pose3d(-tagSpacePose[2], tagSpacePose[0], -tagSpacePose[1],
                new Rotation3d(Units.degreesToRadians(tagSpacePose[5]), Units.degreesToRadians(tagSpacePose[3]), 
                -Units.degreesToRadians(tagSpacePose[4] +  180)));
    }

    private static final int kNumMeasurements = 3000;

    private PoseCalibrationInfo calibrationInfo = new PoseCalibrationInfo();
    private boolean didWriteResults = false;
    private final List<Pose3d> limelightPoseMeasurements = new ArrayList<>();
    private final String name;

    public LimelightPoseCalibrator(String jsonFileName) {
        this.name = jsonFileName.replace("Calibration.json", "");
        try {
            final Path jsonFilePath = Paths.get(Filesystem.getDeployDirectory().getPath(), File.separator, jsonFileName);
            final String jsonContent = Files.readString(jsonFilePath);
            calibrationInfo = JSON.parseObject(jsonContent, PoseCalibrationInfo.class);
            if (calibrationInfo == null) {
                System.out.println("Parsed calibration info is null!");
                calibrationInfo = new PoseCalibrationInfo();
            } else {
                System.out.println(String.format("Successfully parsed calibration info in %s. Name: %s, Translation: %s", 
                        jsonFileName, calibrationInfo.limelightName, Arrays.toString(calibrationInfo.robotToTagTranslationInches)));
            }
        } catch(InvalidPathException exception) {
            System.out.println(String.join(" ", "Couldn't create path to JSON file! InvalidPathException:", exception.getMessage()));
        } catch (IOException exception) {
            System.out.println(String.join(" ", "Couldn't read JSON file! IOException:", exception.getMessage()));
        } catch (JSONException exception) {
            System.out.println(String.join(" ", "Couldn't parse JSON file! JSONException:", exception.getMessage()));
        } catch (Exception exception) {
            System.out.println(String.join(" ", "Couldn't read or parse JSON file! Exception:", exception.getMessage()));
        }
    }

    public void start() {
        limelightPoseMeasurements.clear();
        didWriteResults = false;
    }

    public void update() {
        Pose3d cameraPoseInTagSpace = limelightTagSpaceToOurTagSpace(LimelightHelpers.getCameraPose_TargetSpace(calibrationInfo.limelightName));
        limelightPoseMeasurements.add(cameraPoseInTagSpace);
        logPose("Limelight Calibrator/Camera Pose Measurement", cameraPoseInTagSpace);

        if (limelightPoseMeasurements.size() >= kNumMeasurements && !didWriteResults) {
            final Pose3d averageLimelightPoseInTagSpace = getAverageLimelightPose();
            final Pose3d robotPoseInTagSpace = calibrationInfo.getRobotPoseInTagSpaceMeters();
            final Pose3d robotToLimelight = averageLimelightPoseInTagSpace.relativeTo(robotPoseInTagSpace);
            logPose("Limelight Calibrator/Average Limelight Pose", averageLimelightPoseInTagSpace);
            logPose("Limelight Calibrator/Robot Pose in Tag Space", robotPoseInTagSpace);
            logPose("Limelight Calibrator/Limelight Relative to Robot", robotToLimelight);

            System.out.println(String.format("Calibration done for %s! Limelight pose relative to robot is (LL Forward: %f, LL Right: %f, LL Up: %f, LL Roll: %f, LL Pitch: %f, LL Yaw: %f)", 
                    calibrationInfo.limelightName, robotToLimelight.getX(), -robotToLimelight.getY(), robotToLimelight.getZ(), 
                    Units.radiansToDegrees(robotToLimelight.getRotation().getX()), Units.radiansToDegrees(robotToLimelight.getRotation().getY()), Units.radiansToDegrees(robotToLimelight.getRotation().getZ())));
            didWriteResults = true;
        }
    }

    private void logPose(String key, Pose3d pose) {
        Logger.recordOutput(this.name + "/" + key, new double[]{
            Units.metersToInches(pose.getX()),
            Units.metersToInches(pose.getY()),
            Units.metersToInches(pose.getZ()),
            Units.radiansToDegrees(pose.getRotation().getX()),
            Units.radiansToDegrees(pose.getRotation().getY()),
            Units.radiansToDegrees(pose.getRotation().getZ())
        });
    }

    private Pose3d getAverageLimelightPose() {
        double x = 0, y = 0, z = 0, roll = 0, pitch = 0, yaw = 0;

        for (Pose3d pose : limelightPoseMeasurements) {
            x += pose.getX();
            y += pose.getY();
            z += pose.getZ();
            roll += pose.getRotation().getX();
            pitch += pose.getRotation().getY();
            yaw += pose.getRotation().getZ();
        }

        final double numActualMeasurements = limelightPoseMeasurements.size();
        x /= numActualMeasurements;
        y /= numActualMeasurements;
        z /= numActualMeasurements;
        roll /= numActualMeasurements;
        pitch /= numActualMeasurements;
        yaw /= numActualMeasurements;

        return new Pose3d(x, y, z, new Rotation3d(roll, pitch, yaw));
    }
}
