package com.team254.frc2019.controlboard;

public interface IDriveControlBoard {
    double getThrottle();

    double getStrafe();

    double getRotation();

    boolean getDriveLowPower();

    boolean getFieldRelative();

    double getDPad();

    boolean getShoot();

    boolean getThrust();

    boolean getHorizontalAlign();
}