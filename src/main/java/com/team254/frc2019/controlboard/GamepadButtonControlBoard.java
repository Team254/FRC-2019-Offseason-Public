package com.team254.frc2019.controlboard;

import com.team254.frc2019.Constants;
import com.team254.lib.util.Deadband;

public class GamepadButtonControlBoard implements IButtonControlBoard {
    private final double kDeadband = 0.15;
    private static GamepadButtonControlBoard mInstance = null;

    public static GamepadButtonControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new GamepadButtonControlBoard();
        }

        return mInstance;
    }

    private final XboxController mController;

    private GamepadButtonControlBoard() {
        mController = new XboxController(Constants.kButtonGamepadPort);
    }

    @Override
    public boolean getScorePresetLow() {
        return mController.getButton(XboxController.Button.A);
    }

    @Override
    public boolean getScorePresetMiddle() {
        return mController.getButton(XboxController.Button.B);
    }

    @Override
    public boolean getScorePresetHigh() {
        return mController.getButton(XboxController.Button.Y);
    }

    @Override
    public void setRumble(boolean on) {
        mController.setRumble(on);
    }

    @Override
    public boolean getScorePresetCargo() {
        return mController.getButton(XboxController.Button.X);
    }

    @Override
    public boolean getTuck() {
        return mController.getButton(XboxController.Button.LB);
    }

    @Override
    public boolean getPickupDiskWall() {
        return mController.getTrigger(XboxController.Side.RIGHT);
    }

    @Override
    public boolean getPickupBallGround() {
        return mController.getButton(XboxController.Button.RB);
    }

    @Override
    public boolean getToggleHangMode() {
        return mController.getButton(XboxController.Button.START);
    }

    @Override
    public boolean getToggleHangModeLow() {
        return mController.getButton(XboxController.Button.BACK);
    }

    @Override
    public double getJoggingZ() {
        double jog = mController.getJoystick(XboxController.Side.RIGHT, XboxController.Axis.Y);
        if (Deadband.inDeadband(jog, kDeadband)) {
            return 0.0;
        }
        return (jog - kDeadband * Math.signum(jog));
    }
}