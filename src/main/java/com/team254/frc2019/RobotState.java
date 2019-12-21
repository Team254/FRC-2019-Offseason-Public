package com.team254.frc2019;

import com.team254.frc2019.statemachines.SuperstructureCommands;
import com.team254.frc2019.subsystems.EndEffector;
import com.team254.frc2019.subsystems.Limelight;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;
import com.team254.lib.util.MovingAverageTwist2d;
import com.team254.lib.vision.AimingParameters;
import com.team254.lib.vision.GoalTracker;
import com.team254.lib.vision.GoalTracker.TrackReportComparator;
import com.team254.lib.vision.TargetInfo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;

public class RobotState {
    private static RobotState mInstance;

    public static RobotState getInstance() {
        if (mInstance == null) {
            mInstance = new RobotState();
        }

        return mInstance;
    }

    private static final int kObservationBufferSize = 100;

    /*
     * RobotState keeps track of the poses of various coordinate frames throughout
     * the match. A coordinate frame is simply a point and direction in space that
     * defines an (x,y) coordinate system. Transforms (or poses) keep track of the
     * spatial relationship between different frames.
     *
     * Robot frames of interest (from parent to child):
     *
     * 1. Field frame: origin is where the robot is turned on.
     *
     * 2. Vehicle frame: origin is the center of the robot wheelbase, facing
     * forwards
     *
     * As a kinematic chain with 2 frames, there is 1 transform of interest:
     *
     * 1. Field-to-vehicle: This is tracked over time by integrating encoder and
     * gyro measurements. It will inevitably drift, but is usually accurate over
     * short time periods.
     */

    // FPGATimestamp -> Pose2d or Rotation2d
    private InterpolatingTreeMap<InterpolatingDouble, Pose2d> field_to_vehicle_;
    private InterpolatingTreeMap<InterpolatingDouble, Rotation2d> vehicle_to_turret_;
    private Twist2d vehicle_velocity_measured_;
    private MovingAverageTwist2d vehicle_velocity_measured_filtered_;
    private double distance_driven_;

    private Rotation2d kTurretAngle = Rotation2d.fromDegrees(0);

    private GoalTracker vision_target_low_ = new GoalTracker();
    private GoalTracker vision_target_high_ = new GoalTracker();

    List<Translation2d> mCameraToVisionTargetPosesLow = new ArrayList<>();
    List<Translation2d> mCameraToVisionTargetPosesHigh = new ArrayList<>();

    private final double[] kPossibleTargetNormals = {0.0, 90.0, 180.0, 270.0, 30.0, 150.0, 210.0, 330.0};

    private RobotState() {
        reset(0.0, Pose2d.identity());
    }

    public synchronized void reset(double start_time, Pose2d initial_field_to_vehicle) {
        field_to_vehicle_ = new InterpolatingTreeMap<>(kObservationBufferSize);
        field_to_vehicle_.put(new InterpolatingDouble(start_time), initial_field_to_vehicle);

        vehicle_to_turret_ = new InterpolatingTreeMap<>(kObservationBufferSize);
        vehicle_to_turret_.put(new InterpolatingDouble(start_time), kTurretAngle);

        vehicle_velocity_measured_ = Twist2d.identity();
        vehicle_velocity_measured_filtered_ = new MovingAverageTwist2d(25);
        distance_driven_ = 0.0;
    }

    /**
     * Returns the robot's position on the field at a certain time. Linearly
     * interpolates between stored robot positions to fill in the gaps.
     */
    public synchronized Pose2d getFieldToVehicle(double timestamp) {
        return field_to_vehicle_.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Map.Entry<InterpolatingDouble, Pose2d> getLatestFieldToVehicle() {
        return field_to_vehicle_.lastEntry();
    }

    public synchronized void addFieldToVehicleObservation(double timestamp, Pose2d observation) {
        field_to_vehicle_.put(new InterpolatingDouble(timestamp), observation);
    }

    public synchronized Pose2d getPredictedFieldToVehicle(double lookahead_time) {
        Twist2d dpos = vehicle_velocity_measured_filtered_.getAverage().scaled(lookahead_time);
        return getLatestFieldToVehicle().getValue().transformBy(
                new Pose2d(dpos.dx, dpos.dy, Rotation2d.fromRadians(dpos.dtheta))
        );
    }

    public synchronized void addObservations(double timestamp, Twist2d displacement, Twist2d measured_velocity) {
        distance_driven_ += displacement.norm();
        // janky way to ensure the turret always at 0
        addVehicleToTurretObservation(timestamp, kTurretAngle);

        addFieldToVehicleObservation(timestamp,
                Kinematics.integrateForwardKinematics(getLatestFieldToVehicle().getValue(), displacement));
        vehicle_velocity_measured_ = measured_velocity;
        if (Math.abs(vehicle_velocity_measured_.dtheta) < 2.0 * Math.PI) {
            // Reject really high angular velocities from the filter.
            vehicle_velocity_measured_filtered_.add(vehicle_velocity_measured_);
        } else {
            vehicle_velocity_measured_filtered_
                    .add(new Twist2d(vehicle_velocity_measured_.dx, vehicle_velocity_measured_.dy, 0.0));
        }
    }

    public synchronized void addVisionUpdate(double timestamp, List<TargetInfo> observations, Limelight source) {
        mCameraToVisionTargetPosesLow.clear();
        mCameraToVisionTargetPosesHigh.clear();

        if (observations == null || observations.isEmpty()) {
            vision_target_low_.update(timestamp, new ArrayList<>());
            vision_target_high_.update(timestamp, new ArrayList<>());
            return;
        }

        for (TargetInfo target : observations) {
            mCameraToVisionTargetPosesLow.add(getCameraToVisionTargetPose(target, false, source));
            mCameraToVisionTargetPosesHigh.add(getCameraToVisionTargetPose(target, true, source));
        }

        updatePortGoalTracker(timestamp, mCameraToVisionTargetPosesLow, vision_target_low_, source);
        updatePortGoalTracker(timestamp, mCameraToVisionTargetPosesHigh, vision_target_high_, source);
    }


    private void updatePortGoalTracker(double timestamp, List<Translation2d> cameraToVisionTargetPoses, GoalTracker tracker, Limelight source) {
        if (cameraToVisionTargetPoses.size() != 2 ||
                cameraToVisionTargetPoses.get(0) == null ||
                cameraToVisionTargetPoses.get(1) == null) return;
        Pose2d cameraToVisionTarget = Pose2d.fromTranslation(cameraToVisionTargetPoses.get(0).interpolate(
                cameraToVisionTargetPoses.get(1), 0.5));

        Pose2d fieldToVisionTarget = getFieldToTurret(timestamp).transformBy(source.getTurretToLens()).transformBy(cameraToVisionTarget);
        tracker.update(timestamp, List.of(new Pose2d(fieldToVisionTarget.getTranslation(), Rotation2d.identity())));
    }

    public synchronized Pose2d getVehicleToVisionTarget(double timestamp, boolean highTarget) {
        Pose2d fieldToVisionTarget = getFieldToVisionTarget(highTarget);

        if (fieldToVisionTarget == null) {
            return null;
        }

        return getFieldToVehicle(timestamp).inverse().transformBy(fieldToVisionTarget);
    }

    private Translation2d getCameraToVisionTargetPose(TargetInfo target, boolean high, Limelight source) {
        // Compensate for camera pitch
        Translation2d xz_plane_translation = new Translation2d(target.getX(), target.getZ()).rotateBy(source.getHorizontalPlaneToLens());
        double x = xz_plane_translation.x();
        double y = target.getY();
        double z = xz_plane_translation.y();

        // find intersection with the goal
        double differential_height = source.getLensHeight() - (high ? Constants.kPortTargetHeight : Constants.kHatchTargetHeight);
        if ((z < 0.0) == (differential_height > 0.0)) {
            double scaling = differential_height / -z;
            double distance = Math.hypot(x, y) * scaling;
            Rotation2d angle = new Rotation2d(x, y, true);
            return new Translation2d(distance * angle.cos(), distance * angle.sin());
        }

        return null;
    }

    public synchronized Pose2d getFieldToVisionTarget(boolean highTarget) {
        GoalTracker tracker = highTarget ? vision_target_high_ : vision_target_low_;

        if (!tracker.hasTracks()) {
            return null;
        }

        Pose2d fieldToTarget = tracker.getTracks().get(0).field_to_target;

        double normalPositive = (fieldToTarget.getRotation().getDegrees() + 360) % 360;
        double normalClamped = kPossibleTargetNormals[0];
        for (double possible : kPossibleTargetNormals) {
            if (Math.abs(normalPositive - possible) < Math.abs(normalPositive - normalClamped)) {
                normalClamped = possible;
            }
        }

        return new Pose2d(fieldToTarget.getTranslation(), Rotation2d.fromDegrees(normalClamped));
    }

    public synchronized Pose2d getFieldToTurret(double timestamp) {
        return getFieldToVehicle(timestamp).transformBy(Pose2d.fromRotation(getVehicleToTurret(timestamp)));
    }

    public synchronized Rotation2d getVehicleToTurret(double timestamp) {
        return vehicle_to_turret_.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized void addVehicleToTurretObservation(double timestamp, Rotation2d observation) {
        vehicle_to_turret_.put(new InterpolatingDouble(timestamp), observation);
    }

    public synchronized void resetVision() {
        vision_target_low_.reset();
        vision_target_high_.reset();
    }

    public synchronized boolean useHighTarget() {
        return EndEffector.getInstance().getObservedGamePiece() == GamePiece.BALL &&
                !SuperstructureCommands.isInCargoShipPosition();
    }

    public synchronized Optional<AimingParameters> getAimingParameters(boolean highTarget, int prev_track_id, double max_track_age) {
        GoalTracker tracker = highTarget ? vision_target_high_ : vision_target_low_;
        List<GoalTracker.TrackReport> reports = tracker.getTracks();

        if (reports.isEmpty()) {
            return Optional.empty();
        }

        double timestamp = Timer.getFPGATimestamp();

        // Find the best track.
        TrackReportComparator comparator = new TrackReportComparator(
                Constants.kTrackStabilityWeight,
                Constants.kTrackAgeWeight,
                Constants.kTrackSwitchingWeight,
                prev_track_id, timestamp);
        reports.sort(comparator);

        GoalTracker.TrackReport report = null;
        for (GoalTracker.TrackReport track : reports) {
            if (track.latest_timestamp > timestamp - max_track_age) {
                report = track;
                break;
            }
        }
        if (report == null) {
            return Optional.empty();
        }
        Pose2d vehicleToGoal = getFieldToVehicle(timestamp).inverse().transformBy(report.field_to_target).transformBy(getVisionTargetToGoalOffset());

        AimingParameters params = new AimingParameters(vehicleToGoal,
                report.field_to_target,
                report.field_to_target.getRotation(),
                report.latest_timestamp, report.stability, report.id);
        return Optional.of(params);
    }

    public synchronized Pose2d getVisionTargetToGoalOffset() {
        // if (SuperstructureCommands.isInCargoShipPosition() && EndEffector.getInstance().getObservedGamePiece() == GamePiece.BALL) {
        //     return Pose2d.fromTranslation(new Translation2d(-6.0, 0.0));
        // }
        return Pose2d.identity();
    }

    public synchronized double getDistanceDriven() {
        return distance_driven_;
    }

    public synchronized void resetDistanceDriven() {
        distance_driven_ = 0.0;
    }

    public synchronized Twist2d getMeasuredVelocity() {
        return vehicle_velocity_measured_;
    }

    public synchronized Twist2d getSmoothedVelocity() {
        return vehicle_velocity_measured_filtered_.getAverage();
    }

    public synchronized void outputToSmartDashboard() {
        SmartDashboard.putString("Robot Velocity", getMeasuredVelocity().toString());
        SmartDashboard.putString("Robot Pose", getLatestFieldToVehicle().getValue().toString());
    }
}