package com.team254.lib.control;

import com.team254.frc2019.Constants;
import com.team254.frc2019.subsystems.Drive;
import com.team254.lib.util.SynchronousPIDF;

/**
 * Controls overall swerve heading of the robot through motion profile.
 * <p>
 * All units are in degrees (for this class only) for easy integration with DPad
 */
public class SwerveHeadingController {
    private static SwerveHeadingController mInstance;

    public static SwerveHeadingController getInstance() {
        if (mInstance == null) {
            mInstance = new SwerveHeadingController();
        }

        return mInstance;
    }

    public enum HeadingControllerState {
        OFF, SNAP, // for dpad snapping to cardinals
        MAINTAIN, // maintaining current heading while driving
    }

    private final SynchronousPIDF mPIDFController;
    private double mSetpoint = 0.0;

    private HeadingControllerState mHeadingControllerState = HeadingControllerState.OFF;

    private SwerveHeadingController() {
        mPIDFController = new SynchronousPIDF();
    }

    public HeadingControllerState getHeadingControllerState() {
        return mHeadingControllerState;
    }

    public void setHeadingControllerState(HeadingControllerState state) {
        mHeadingControllerState = state;
    }

    /**
     * @param goal pos in degrees
     */
    public void setGoal(double goal_pos) {
        mSetpoint = goal_pos;
    }

    public boolean isAtGoal() {
        return mPIDFController.onTarget(Constants.kSwerveHeadingControllerErrorTolerance);
    }

    /**
     * Should be called from a looper at a constant dt
     */
    public double update() {
        mPIDFController.setSetpoint(mSetpoint);
        double current_angle = Drive.getInstance().getHeading().getDegrees();
        double current_error = mSetpoint - current_angle;

        if (current_error > 180) {
            current_angle += 360;
        } else if (current_error < -180) {
            current_angle -= 360;
        }

        switch (mHeadingControllerState) {
            case OFF:
                return 0.0;
            case SNAP:
                mPIDFController.setPID(Constants.kSnapSwerveHeadingKp, Constants.kSnapSwerveHeadingKi, Constants.kSnapSwerveHeadingKd);
                break;
            case MAINTAIN:
                mPIDFController.setPID(Constants.kMaintainSwerveHeadingKp, Constants.kMaintainSwerveHeadingKi, Constants.kMaintainSwerveHeadingKd);
                break;
        }

        return mPIDFController.calculate(current_angle);
    }
}