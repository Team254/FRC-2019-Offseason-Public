package com.team254.frc2019.controlboard;

import com.team254.frc2019.Constants;
import com.team254.lib.geometry.Rotation2d;

public class GamepadDriveControlBoard implements IDriveControlBoard {
    private static GamepadDriveControlBoard mInstance = null;

    public static GamepadDriveControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new GamepadDriveControlBoard();
        }

        return mInstance;
    }

    private final XboxController mController;

    private GamepadDriveControlBoard() {
        mController = new XboxController(Constants.kDriveGamepadPort);
    }

    @Override
    public double getThrottle() {
        return mController.getJoystick(XboxController.Side.LEFT, XboxController.Axis.Y);
    }

    @Override
    public double getStrafe() {
        return -mController.getJoystick(XboxController.Side.LEFT, XboxController.Axis.X);
    }

    @Override
    public double getRotation() {
        return -mController.getJoystick(XboxController.Side.RIGHT, XboxController.Axis.X);
    }

    @Override
    public boolean getDriveLowPower() {
        return mController.getButton(XboxController.Button.A);
    }

    @Override
    public boolean getFieldRelative() {
        return !mController.getButton(XboxController.Button.LB);
    }

    @Override
    public double getDPad() {

        if (mController.getDPad() == -1) {
            return -1;
        }

        if (mController.getTrigger(XboxController.Side.LEFT)) {
            double degs = SwerveCardinal.findClosest(Rotation2d.fromDegrees(mController.getDPad()), true).rotation.getDegrees();
            if (degs < 0) {
                degs += 360;
            }
            return degs;
        } else {
            double degs = SwerveCardinal.findClosest(Rotation2d.fromDegrees(mController.getDPad()), false).rotation.getDegrees();
            if (degs < 0) {
                degs += 360;
            }
            return degs;
        }
    }


    enum SwerveCardinal {
        BACK(180),
        FRONT(0),
        LEFT(90, -90, false),
        RIGHT(-90, 90, false),
        NONE(0),
        FRONT_LEFT(-30, 180, true),
        FRONT_RIGHT(-150, 90, true),
        BACK_LEFT(150, 0, true),
        BACK_RIGHT(30, -90, true);

        public final Rotation2d rotation;
        private final Rotation2d inputDirection;
        private final boolean isARocketCardinal;

        SwerveCardinal(double degrees) {
            this(degrees, degrees, false);
        }

        SwerveCardinal(double degrees, double inputDirectionDegrees, boolean isARocketCardinal) {
            rotation = Rotation2d.fromDegrees(degrees);
            inputDirection = Rotation2d.fromDegrees(inputDirectionDegrees);
            this.isARocketCardinal = isARocketCardinal;
        }

        public static SwerveCardinal findClosest(double xAxis, double yAxis, boolean isARocketCardinal) {
            return findClosest(new Rotation2d(yAxis, -xAxis, true), isARocketCardinal);
        }

        public static SwerveCardinal findClosest(Rotation2d stickDirection, boolean isARocketCardinal) {
            var values = SwerveCardinal.values();

            SwerveCardinal closest = null;
            double closestDistance = Double.MAX_VALUE;
            for (int i = 0; i < values.length; i++) {
                if (values[i].isARocketCardinal != isARocketCardinal) {
                    continue;
                }
                var checkDirection = values[i];
                var distance = Math.abs(stickDirection.distance(checkDirection.inputDirection));
                if (distance < closestDistance) {
                    closestDistance = distance;
                    closest = checkDirection;
                }
            }
            return closest;
        }

        public static boolean isDiagonal(SwerveCardinal cardinal) {
            return cardinal == FRONT_LEFT || cardinal == FRONT_RIGHT || cardinal == BACK_LEFT || cardinal == BACK_RIGHT;
        }
    }

    @Override
    public boolean getThrust() {
        return mController.getButton(XboxController.Button.RB);
    }

    @Override
    public boolean getShoot() {
        return mController.getTrigger(XboxController.Side.RIGHT);
    }

    @Override
    public boolean getHorizontalAlign() {
        return mController.getButton(XboxController.Button.B);
    }
}