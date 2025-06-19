package com.team5817.frc2025;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import java.util.Map;
import java.util.Optional;


import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;
import com.team254.lib.util.MovingAverageTwist2d;
import com.team5817.frc2025.subsystems.Vision.VisionPoseAcceptor;
import com.team5817.lib.util.UnscentedKalmanFilter;

/**
 * The RobotState class tracks the robot's position on the field using odometry and vision updates.
 */
public class RobotState {
    // Existing member variables
    private static RobotState mInstance;
    private Optional<VisionUpdate> mLatestVisionUpdate;
    private UnscentedKalmanFilter<N2, N2, N2> mKalmanFilter;
    private VisionPoseAcceptor mPoseAcceptor;
    private Pose2d mDisplayVisionPose;
    private boolean mHasBeenEnabled = false;


    private static final int kObservationBufferSize = 50;
    private Optional<Translation2d> initialPoseError = Optional.empty();
    private InterpolatingTreeMap<InterpolatingDouble, Pose2d> poseFromOdom;
    private InterpolatingTreeMap<InterpolatingDouble, Translation2d> visionPoseComponent;

    private Twist2d PredictedVelocity;
    private Twist2d MeasuredVelocity;
    private MovingAverageTwist2d filteredMeasuredVelocity;

    /**
     * Returns the singleton instance of RobotState.
     * 
     * @return the singleton instance of RobotState
     */
    public static RobotState getInstance() {
        if (mInstance == null)

            mInstance = new RobotState();
        return mInstance;
    }

    /**
     * Private constructor to initialize the RobotState.
     */
    private RobotState() {
        reset(0.0, Pose2d.fromTranslation(new Translation2d(0, 0)));
    }

    /**
     * Resets the RobotState with the given start time and initial pose.
     * 
     * @param start_time the start time
     * @param initialPose the initial pose
     */
    public void reset(double start_time, Pose2d initialPose) {
        resetKalmanFilters();
        poseFromOdom = new InterpolatingTreeMap<>(kObservationBufferSize);
        poseFromOdom.put(new InterpolatingDouble(start_time), initialPose);
        visionPoseComponent = new InterpolatingTreeMap<>(kObservationBufferSize);
        visionPoseComponent.put(new InterpolatingDouble(start_time), Translation2d.identity());

        PredictedVelocity = Twist2d.identity();
        MeasuredVelocity = Twist2d.identity();
        filteredMeasuredVelocity = new MovingAverageTwist2d(25);
        mLatestVisionUpdate = Optional.empty();
        mDisplayVisionPose = Pose2d.identity();
        mPoseAcceptor = new VisionPoseAcceptor();
        initialPoseError = Optional.empty();

    }

    /**
     * Resets the RobotState with the current time and identity pose.
     */
    public void reset() {
        reset(Timer.getFPGATimestamp(), Pose2d.identity());
    }

    /**
     * Resets the Kalman filters.
     */
    public void resetKalmanFilters() {
        mKalmanFilter = new UnscentedKalmanFilter<>(
                Nat.N2(),
                Nat.N2(),
                (x, u) -> VecBuilder.fill(0.0, 0.0),
                (x, u) -> x,
                RobotConstants.kStateStdDevs,
                RobotConstants.kLocalMeasurementStdDevs, .01);

    }

    /**
     * Returns whether the robot has been enabled.
     * 
     * @return true if the robot has been enabled, false otherwise
     */
    public boolean getHasBeenEnabled() {
        return mHasBeenEnabled;
    }

    /**
     * Sets whether the robot has been enabled.
     * 
     * @param hasBeenEnabled true if the robot has been enabled, false otherwise
     */
    public void setHasBeenEnabled(boolean hasBeenEnabled) {
        mHasBeenEnabled = hasBeenEnabled;
    }

    /**
     * Returns the robot's position on the field at a certain time. Linearly
     * interpolates between stored robot positions
     * to fill in the gaps.
     * 
     * @param timestamp the timestamp
     * @return the robot's position on the field
     */
    public Pose2d getPoseFromOdom(double timestamp) {
        return poseFromOdom.getInterpolated(new InterpolatingDouble(timestamp));
    }

    /**
     * Returns the latest pose from odometry.
     * 
     * @return the latest pose from odometry
     */
    public Map.Entry<InterpolatingDouble, Pose2d> getLatestPoseFromOdom() {
        return poseFromOdom.lastEntry();
    }

    /**
     * Adds a pose observation.
     * 
     * @param timestamp the timestamp
     * @param observation the pose observation
     */
    public void addPoseObservation(double timestamp, Pose2d observation) {
        poseFromOdom.put(new InterpolatingDouble(timestamp), observation);
    }

    /**
     * Adds odometry observations.
     * 
     * @param timestamp the timestamp
     * @param poseFromOdom the pose from odometry
     * @param measured_velocity the measured velocity
     * @param predicted_velocity the predicted velocity
     */
    public void addOdomObservations(double timestamp, Pose2d poseFromOdom, Twist2d measured_velocity,
            Twist2d predicted_velocity) {
        mKalmanFilter.predict(VecBuilder.fill(0.0, 0.0), .01);

        addPoseObservation(timestamp, poseFromOdom);
        MeasuredVelocity = measured_velocity;
        filteredMeasuredVelocity.add(MeasuredVelocity);
        PredictedVelocity = new Twist2d(predicted_velocity.dx, -predicted_velocity.dy, predicted_velocity.dtheta);
    }

    /**
     * Returns the predicted velocity.
     * 
     * @return the predicted velocity
     */
    public Twist2d getPredictedVelocity() {
        return PredictedVelocity;
    }

    /**
     * Returns the measured velocity.
     * 
     * @return the measured velocity
     */
    public Twist2d getMeasuredVelocity() {
        return MeasuredVelocity;
    }

    /**
     * Returns the smoothed velocity.
     * 
     * @return the smoothed velocity
     */
    public Twist2d getSmoothedVelocity() {
        return filteredMeasuredVelocity.getAverage();
    }


    /**
     * Adds a Vision Update
     * 
     * @param visionUpdate the vision update
     */
    public void addVisionUpdate(VisionUpdate visionUpdate) {
        // Use the same logic for both methods.
        if (!mLatestVisionUpdate.isEmpty() || initialPoseError.isPresent()) {
            // Get the Timestamp of the Vision Reading
            double visionTimestamp = mLatestVisionUpdate.get().getTimestamp();

            // Get pose from odometry based on vision timestamp
            Pose2d odomToVehicle = getPoseFromOdom(visionTimestamp);
            Pose2d visionFieldToVehicle = Pose2d.fromTranslation(visionUpdate.getFieldToVision());

            // Check if the vision update should be accepted
            if (!mPoseAcceptor.shouldAcceptVision(visionTimestamp, visionFieldToVehicle, getLatestGlobalKalmanPose(),
                    MeasuredVelocity, false)) {
                return;
            }

            // Calculate the vision odometry error and apply it to the Kalman filter
            Translation2d visionOdomError = visionFieldToVehicle.getTranslation()
                    .translateBy(odomToVehicle.getTranslation().inverse());
            mDisplayVisionPose = visionFieldToVehicle;

            try {
                // Use the correct Kalman filter depending on the method
                mKalmanFilter.correct(VecBuilder.fill(0.0, 0.0),
                        VecBuilder.fill(visionOdomError.x(), visionOdomError.y()));
                visionPoseComponent.put(new InterpolatingDouble(visionTimestamp),
                        Pose2d.fromTranslation(new Translation2d(mKalmanFilter.getXhat(0), mKalmanFilter.getXhat(1)))
                                .getTranslation());
            } catch (Exception e) {
                DriverStation.reportError("QR Decomposition failed: ", e.getStackTrace());
            }
        } else {
            // Handle case when there's no recent vision update or initial pose error
            double visionTimestamp = visionUpdate.getTimestamp();
            Pose2d proximatePose = poseFromOdom.getInterpolated(new InterpolatingDouble(visionTimestamp));

            Translation2d fieldToVision = visionUpdate.getFieldToVision();
            Translation2d odomToVehicleTranslation = proximatePose.getTranslation();
            Translation2d fieldToOdom = fieldToVision.translateBy(odomToVehicleTranslation.inverse());

            visionPoseComponent.put(new InterpolatingDouble(visionTimestamp), fieldToOdom);
            initialPoseError = Optional.of(fieldToOdom);

            mKalmanFilter.setXhat(0, fieldToOdom.x());
            mKalmanFilter.setXhat(1, fieldToOdom.y());
        }

        // Update the latest vision update
        mLatestVisionUpdate = Optional.ofNullable(visionUpdate);
    }

    /**
     * Returns the display vision pose.
     * 
     * @return the display vision pose
     */
    public Pose2d getDisplayVisionPose() {
        if (mDisplayVisionPose == null) {
            return Pose2d.fromTranslation(Translation2d.identity()); // Out of frame
        }
        return mDisplayVisionPose;
    }

    /**
     * Return Initial Vision Offset for Pure Odometry Visualization Purposes
     * 
     * @return the initial global pose error
     */
    public Pose2d getInitialGlobalPoseError() {
        if (initialPoseError.isEmpty())
            return Pose2d.identity();
        return Pose2d.fromTranslation(initialPoseError.get());
    }

    /**
     * Returns the global vision pose component at a given timestamp.
     * 
     * @param timestamp the timestamp
     * @return the global vision pose component
     */
    public Translation2d getGlobalVisionPoseComponent(double timestamp) {
        if (initialPoseError.isEmpty())
            return Translation2d.identity();
        return initialPoseError.get().inverse()
                .translateBy(visionPoseComponent.getInterpolated(new InterpolatingDouble(timestamp)));
    }

    /**
     * Returns the global absolute vision pose component at a given timestamp.
     * 
     * @param timestamp the timestamp
     * @return the global absolute vision pose component
     */
    public Translation2d getGlobalAbsoluteVisionPoseComponent(double timestamp) {
        return visionPoseComponent.getInterpolated(new InterpolatingDouble(timestamp));
    }

    /**
     * Returns the latest global vision pose component.
     * 
     * @return the latest global vision pose component
     */
    public Translation2d getLatestGlobalVisionPoseComponent() {
        return getGlobalAbsoluteVisionPoseComponent(visionPoseComponent.lastKey().value);
    }

    /**
     * Get Current Field to Vehicle using Filter Idea 1 (Offset Space) => Add the
     * Offset outputted by the Filter to Current Odom
     * 
     * @param timestamp the timestamp
     * @return the global Kalman pose
     */
    public Pose2d getGlobalKalmanPose(double timestamp) {
        Pose2d poseFromOdom = getPoseFromOdom(timestamp);

        Translation2d kalmanPoseOffset = getGlobalAbsoluteVisionPoseComponent(timestamp);
        return new Pose2d(kalmanPoseOffset.translateBy(poseFromOdom.getTranslation()), poseFromOdom.getRotation());

    }

    /**
     * Returns the latest global Kalman pose.
     * 
     * @return the latest global Kalman pose
     */
    public Pose2d getLatestGlobalKalmanPose() {
        Pose2d poseFromOdom = getLatestPoseFromOdom().getValue();
        return new Pose2d(getLatestGlobalVisionPoseComponent().getTranslation().add(poseFromOdom.getTranslation()),
                poseFromOdom.getRotation());
    }

    /**
     * Returns the latest global vision update.
     * 
     * @return the latest global vision update
     */
    public Optional<VisionUpdate> getLatestGlobalVisionUpdate() {
        return mLatestVisionUpdate;
    }

    /**
     * The VisionUpdate class represents a vision update with a timestamp, target area, and field-to-vision translation.
     */
    public static class VisionUpdate {
        private double timestamp;
        private double ta;
        private Translation2d field_to_vision;
        private int mID;

        /**
         * Constructs a VisionUpdate.
         * 
         * @param id the ID of the vision update
         * @param timestamp the timestamp of the vision update
         * @param ta the target area
         * @param field_to_vision the field-to-vision translation
         */
        public VisionUpdate(int id, double timestamp, double ta, Translation2d field_to_vision) {
            this.mID = id;
            this.ta = ta;
            this.timestamp = timestamp;
            this.field_to_vision = field_to_vision;
        }

        /**
         * Returns the timestamp of the vision update.
         * 
         * @return the timestamp
         */
        public double getTimestamp() {
            return timestamp;
        }

        /**
         * Returns the field-to-vision translation.
         * 
         * @return the field-to-vision translation
         */
        public Translation2d getFieldToVision() {
            return field_to_vision;
        }

        /**
         * Returns the target area.
         * 
         * @return the target area
         */
        public double getTa() {
            return ta;
        }

        /**
         * Returns the ID of the vision update.
         * 
         * @return the ID
         */
        public Integer getID() {
            return mID;
        }
    }
}