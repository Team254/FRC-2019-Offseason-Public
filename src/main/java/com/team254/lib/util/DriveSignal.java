package com.team254.lib.util;

import com.team254.lib.geometry.Rotation2d;

import java.text.DecimalFormat;

/**
 * A drivetrain signal containing the speed and azimuth for each wheel
 */
public class DriveSignal {
    private double[] mWheelSpeeds;
    private Rotation2d[] mWheelAzimuths; // Radians!

    public DriveSignal() {
        this(new double[]{0, 0, 0, 0}, new Rotation2d[]{Rotation2d.identity(), Rotation2d.identity(), Rotation2d.identity(), Rotation2d.identity()});
    }

    public DriveSignal(double[] wheelSpeeds, Rotation2d[] wheelAzimuths) {
        mWheelSpeeds = wheelSpeeds;
        mWheelAzimuths = wheelAzimuths;
    }

    public double[] getWheelSpeeds() {
        return mWheelSpeeds;
    }

    public Rotation2d[] getWheelAzimuths() {
        return mWheelAzimuths;
    }

    @Override
    public String toString() {
        String ret_val = "DriveSignal - \n";
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        for (int i = 0; i < mWheelSpeeds.length; i++) {
            ret_val += "\tWheel " + i + ": Speed - " + mWheelSpeeds[i] + ", Azimuth - " + fmt.format(mWheelAzimuths[i].getDegrees()) + " deg\n";
        }

        return ret_val;
    }
}