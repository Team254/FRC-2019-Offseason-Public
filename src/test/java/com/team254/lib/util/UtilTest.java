package com.team254.lib.util;


import org.junit.Assert;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.JUnit4;

/**
 * Class that tests the system test
 */
@RunWith(JUnit4.class)
public class UtilTest {
    @Test
    public void testBounding() {
        Assert.assertEquals(0, Util.bound0To2PIRadians(4 * Math.PI), Util.kEpsilon);
        Assert.assertEquals(Math.PI / 2, Util.bound0To2PIRadians(5 * Math.PI / 2), Util.kEpsilon);
        Assert.assertEquals(7 * Math.PI / 8, Util.bound0To2PIRadians(-25 * Math.PI / 8), Util.kEpsilon);
        Assert.assertEquals(3 * Math.PI / 2, Util.bound0To2PIRadians(-5 * Math.PI / 2), Util.kEpsilon);
    }

}