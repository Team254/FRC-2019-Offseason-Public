package com.team254.frc2019;

import com.team254.frc2019.subsystems.Drive;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.util.DriveSignal;

/**
 * Provides forward and inverse kinematics equations for the robot modeling the
 * wheelbase as a swerve drive.
 * <p>
 * Equations extrapolated through papers by Ether, then corrected for our frame of reference:
 * https://www.chiefdelphi.com/t/paper-4-wheel-independent-drive-independent-steering-swerve/107383
 */

public class Kinematics {
    private static final double L = Constants.kDriveWheelbase;
    private static final double W = Constants.kDriveTrackwidth;
    private static final double R = Math.hypot(L, W);

    /**
     * Forward kinematics using only encoders
     */
    public static Twist2d forwardKinematics(DriveSignal drive_signal) {
        return forwardKinematics(drive_signal.getWheelSpeeds(), drive_signal.getWheelAzimuths());
    }

    /**
     * @param wheel_speeds
     * @param wheel_azimuths
     * @return Twist2d representing forward, strafe, and angular velocities in real world units
     */
    public static Twist2d forwardKinematics(double[] wheel_speeds, Rotation2d[] wheel_azimuths) {
        double[] vx = new double[4]; // wheel velocities in the x (forward) direction
        double[] vy = new double[4]; // wheel velocities in the y (strafe) direction

        for (int i = 0; i < vx.length; i++) {
            vx[i] = wheel_azimuths[i].cos() * wheel_speeds[i];
            vy[i] = wheel_azimuths[i].sin() * wheel_speeds[i];
        }

        // average possible solutions to minimize error
        double A = (vy[2] + vy[3]) / 2;
        double B = (vy[0] + vy[1]) / 2;
        double C = (vx[0] + vx[3]) / 2;
        double D = (vx[1] + vx[2]) / 2;

        // average possible solutions to minimize error
        double forward = (C + D) / 2;
        double strafe = (A + B) / 2;
        double rotation = (((strafe - A) * R / L) + ((B - strafe) * R / L) + ((forward - C) * R / W)
                + ((D - forward) * R / W)) / 4;

        return new Twist2d(forward, strafe, rotation);
    }

    /**
     * Use Gyro for dtheta
     */
    public static Twist2d forwardKinematics(DriveSignal drive_signal, Rotation2d prev_heading,
                                            Rotation2d current_heading, double dt) {
        Twist2d ret_val = forwardKinematics(drive_signal);
        return new Twist2d(ret_val.dx, ret_val.dy, prev_heading.inverse().rotateBy(current_heading).getRadians() / dt);
    }

    public static Twist2d forwardKinematics(double[] wheel_speeds, Rotation2d[] wheel_azimuths, Rotation2d prev_heading,
                                            Rotation2d current_heading, double dt) {
        Twist2d ret_val = forwardKinematics(wheel_speeds, wheel_azimuths);
        return new Twist2d(ret_val.dx, ret_val.dy, prev_heading.inverse().rotateBy(current_heading).getRadians() / dt);
    }

    /**
     * For convenience, integrate forward kinematics with a Twist2d and previous
     * rotation.
     */
    public static Pose2d integrateForwardKinematics(Pose2d current_pose, Twist2d forward_kinematics) {
        return current_pose.transformBy(new Pose2d(forward_kinematics.dx, forward_kinematics.dy,
                Rotation2d.fromRadians(forward_kinematics.dtheta)));
    }

    public static DriveSignal inverseKinematics(double forward, double strafe, double rotation,
                                                boolean field_relative) {
        return inverseKinematics(forward, strafe, rotation, field_relative, true);
    }

    public static DriveSignal inverseKinematics(double forward, double strafe, double rotation, boolean field_relative,
                                                boolean normalize_outputs) {
        if (field_relative) {
            Rotation2d gyroHeading = Drive.getInstance().getHeading();
            double temp = forward * gyroHeading.cos() + strafe * gyroHeading.sin();
            strafe = -forward * gyroHeading.sin() + strafe * gyroHeading.cos();
            forward = temp;
        }

        double A = strafe - rotation * L / R;
        double B = strafe + rotation * L / R;
        double C = forward - rotation * W / R;
        double D = forward + rotation * W / R;

        double[] wheel_speeds = new double[4];
        wheel_speeds[0] = Math.hypot(B, C);
        wheel_speeds[1] = Math.hypot(B, D);
        wheel_speeds[2] = Math.hypot(A, D);
        wheel_speeds[3] = Math.hypot(A, C);

        // normalize wheel speeds if above 1
        if (normalize_outputs) {
            double max_speed = 1;
            for (int i = 0; i < wheel_speeds.length; i++) {
                if (Math.abs(wheel_speeds[i]) > max_speed) {
                    max_speed = Math.abs(wheel_speeds[i]);
                }
            }

            for (var i = 0; i < wheel_speeds.length; i++) {
                wheel_speeds[i] /= max_speed;
            }
        }

        Rotation2d[] wheel_azimuths = new Rotation2d[4];
        wheel_azimuths[0] = Rotation2d.fromRadians(Math.atan2(B, C));
        wheel_azimuths[1] = Rotation2d.fromRadians(Math.atan2(B, D));
        wheel_azimuths[2] = Rotation2d.fromRadians(Math.atan2(A, D));
        wheel_azimuths[3] = Rotation2d.fromRadians(Math.atan2(A, C));

        return new DriveSignal(wheel_speeds, wheel_azimuths);
    }
}