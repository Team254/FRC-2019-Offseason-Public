package com.team254.frc2019.controlboard;

public interface IButtonControlBoard {
    boolean getScorePresetLow();

    boolean getScorePresetMiddle();

    boolean getScorePresetHigh();

    boolean getScorePresetCargo();

    boolean getTuck();

    boolean getPickupDiskWall();

    boolean getPickupBallGround();

    void setRumble(boolean on);

    // Climbing
    boolean getToggleHangMode();

    boolean getToggleHangModeLow();

    double getJoggingZ();
}