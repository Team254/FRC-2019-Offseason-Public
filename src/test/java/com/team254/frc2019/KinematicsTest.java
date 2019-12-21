package com.team254.frc2019;

import com.team254.lib.geometry.Twist2d;
import com.team254.lib.util.Util;
import org.junit.Assert;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.JUnit4;

/**
 * Class that tests the system test
 */
@RunWith(JUnit4.class)
public class KinematicsTest {
    public final double R = Math.hypot(Constants.kDriveWheelbase, Constants.kDriveTrackwidth);

    public void testKinematics(double forwardInput, double strafeInput, double rotationInput) {
        Twist2d result = Kinematics.forwardKinematics(Kinematics.inverseKinematics(forwardInput, strafeInput, rotationInput, false, false));

        Assert.assertEquals(forwardInput, result.dx, Util.kEpsilon);
        Assert.assertEquals(strafeInput, result.dy, Util.kEpsilon);
        Assert.assertEquals(rotationInput, result.dtheta, Util.kEpsilon);
    }

    @Test
    public void test() {
        testKinematics(1, 0, 0);
        testKinematics(0, 1, 0);
        testKinematics(0, 0, 1);
        testKinematics(0.6, 0.4, 0.5);
    }
}