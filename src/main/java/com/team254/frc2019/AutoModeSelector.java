package com.team254.frc2019;

import com.team254.frc2019.auto.modes.AutoModeBase;
import com.team254.frc2019.auto.modes.DriveByCameraMode;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

public class AutoModeSelector {

    enum StartingPosition {
        LEFT_HAB_2, RIGHT_HAB_2, CENTER_HAB_1, LEFT_HAB_1, RIGHT_HAB_1
    }

    enum DesiredMode {
        DRIVE_BY_CAMERA
    }

    private DesiredMode mCachedDesiredMode = null;
    private StartingPosition mCachedStartingPosition = null;

    private SendableChooser<DesiredMode> mModeChooser;
    private SendableChooser<StartingPosition> mStartPositionChooser;

    private Optional<AutoModeBase> mAutoMode = Optional.empty();

    public AutoModeSelector() {
        mStartPositionChooser = new SendableChooser<>();
        mStartPositionChooser.setDefaultOption("Left HAB 2", StartingPosition.LEFT_HAB_2);
        mStartPositionChooser.addOption("Right HAB 2", StartingPosition.RIGHT_HAB_2);
        mStartPositionChooser.addOption("Right HAB 1", StartingPosition.RIGHT_HAB_1);
        mStartPositionChooser.addOption("Left HAB 1", StartingPosition.LEFT_HAB_1);
        mStartPositionChooser.addOption("Center HAB 1", StartingPosition.CENTER_HAB_1);

        SmartDashboard.putData("Starting Position", mStartPositionChooser);

        mModeChooser = new SendableChooser<>();
        mModeChooser.setDefaultOption("Drive By Camera", DesiredMode.DRIVE_BY_CAMERA);
        SmartDashboard.putData("Auto mode", mModeChooser);
    }

    public void updateModeCreator() {
        DesiredMode desiredMode = mModeChooser.getSelected();
        StartingPosition staringPosition = mStartPositionChooser.getSelected();
        if (mCachedDesiredMode != desiredMode || staringPosition != mCachedStartingPosition) {
            System.out.println("Auto selection changed, updating creator: desiredMode->" + desiredMode.name()
                    + ", starting position->" + staringPosition.name());
            mAutoMode = getAutoModeForParams(desiredMode, staringPosition);
        }
        mCachedDesiredMode = desiredMode;
        mCachedStartingPosition = staringPosition;
    }

    private boolean startingLeft(StartingPosition position) {
        return position == StartingPosition.LEFT_HAB_1 || position == StartingPosition.LEFT_HAB_2;
    }

    private boolean startingHab1(StartingPosition position) {
        return position == StartingPosition.LEFT_HAB_1 || position == StartingPosition.RIGHT_HAB_1;
    }

    private Optional<AutoModeBase> getAutoModeForParams(DesiredMode mode, StartingPosition position) {
        switch (mode) {
            case DRIVE_BY_CAMERA:
                return Optional.of(new DriveByCameraMode());
            default:
                break;
        }

        System.err.println("No valid auto mode found for  " + mode);
        return Optional.empty();
    }

    public void reset() {
        mAutoMode = Optional.empty();
        mCachedDesiredMode = null;
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putString("AutoModeSelected", mCachedDesiredMode.name());
        SmartDashboard.putString("StartingPositionSelected", mCachedStartingPosition.name());
    }

    public Optional<AutoModeBase> getAutoMode() {
        return mAutoMode;
    }

    public boolean isDriveByCamera() {
        return mCachedDesiredMode == DesiredMode.DRIVE_BY_CAMERA;
    }
}