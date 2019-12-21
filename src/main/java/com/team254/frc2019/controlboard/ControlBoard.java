package com.team254.frc2019.controlboard;

public class ControlBoard implements IControlBoard {
    private static ControlBoard mInstance = null;

    public static ControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new ControlBoard();
        }

        return mInstance;
    }

    private final IDriveControlBoard mDriveControlBoard;
    private final IButtonControlBoard mButtonControlBoard;

    private ControlBoard() {
        mDriveControlBoard = GamepadDriveControlBoard.getInstance();
        mButtonControlBoard = GamepadButtonControlBoard.getInstance();
    }

    @Override
    public double getThrottle() {
        return mDriveControlBoard.getThrottle();
    }

    @Override
    public double getStrafe() {
        return mDriveControlBoard.getStrafe();
    }

    @Override
    public double getRotation() {
        return mDriveControlBoard.getRotation();
    }

    @Override
    public boolean getDriveLowPower() {
        return mDriveControlBoard.getDriveLowPower();
    }

    @Override
    public boolean getFieldRelative() {
        return mDriveControlBoard.getFieldRelative();
    }

    @Override
    public double getDPad() {
        return mDriveControlBoard.getDPad();
    }

    @Override
    public boolean getShoot() {
        return mDriveControlBoard.getShoot();
    }

    @Override
    public boolean getThrust() {
        return mDriveControlBoard.getThrust();
    }

    @Override
    public boolean getScorePresetLow() {
        return mButtonControlBoard.getScorePresetLow();
    }

    @Override
    public boolean getScorePresetMiddle() {
        return mButtonControlBoard.getScorePresetMiddle();
    }

    @Override
    public boolean getScorePresetHigh() {
        return mButtonControlBoard.getScorePresetHigh();
    }

    @Override
    public boolean getScorePresetCargo() {
        return mButtonControlBoard.getScorePresetCargo();
    }

    @Override
    public boolean getTuck() {
        return mButtonControlBoard.getTuck();
    }

    @Override
    public boolean getPickupDiskWall() {
        return mButtonControlBoard.getPickupDiskWall();
    }

    @Override
    public boolean getPickupBallGround() {
        return mButtonControlBoard.getPickupBallGround();
    }

    @Override
    public void setRumble(boolean on) {
        mButtonControlBoard.setRumble(on);
    }

    @Override
    public boolean getToggleHangMode() {
        return mButtonControlBoard.getToggleHangMode();
    }

    @Override
    public boolean getToggleHangModeLow() {
        return mButtonControlBoard.getToggleHangModeLow();
    }

    @Override
    public double getJoggingZ() {
        return mButtonControlBoard.getJoggingZ();
    }

    @Override
    public boolean getHorizontalAlign() {
        return mDriveControlBoard.getHorizontalAlign();
    }
}
