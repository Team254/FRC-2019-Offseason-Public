package com.team254.frc2019.subsystems;

import com.team254.frc2019.Constants;
import com.team254.frc2019.RobotState;
import com.team254.frc2019.loops.ILooper;
import com.team254.frc2019.loops.Loop;
import com.team254.frc2019.planners.TuckPlanner;
import com.team254.frc2019.states.SuperstructureGoal;
import com.team254.frc2019.states.SuperstructureState;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.Units;
import com.team254.lib.util.Util;
import com.team254.lib.vision.AimingParameters;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

/**
 * The superstructure subsystem is the overarching class containing all components of the superstructure: the
 * elevator, arm, and wrist. The superstructure subsystem also uses info from the vision system.
 * <p>
 * Instead of interacting individually with subsystems like the elevator and arm, the {@link Robot} class sends commands
 * to the superstructure, which individually decides how to move each subsystem to get there.
 * <p>
 * The Superstructure class also adjusts the overall goal based on the elevator control modes.
 *
 * @see com.team254.frc2019.statemachines.SuperstructureCommands
 */
public class Superstructure extends Subsystem {
    private static Superstructure mInstance;

    private final Elevator mElevator = Elevator.getInstance();
    private final Arm mArm = Arm.getInstance();
    private final Wrist mWrist = Wrist.getInstance();
    private final RobotState mRobotState = RobotState.getInstance();

    // Current state = actual measured state of each DOF.
    private SuperstructureState mCurrentState = new SuperstructureState();
    // Current setpoint = output of the planner. May or may not be the final goal.
    private SuperstructureGoal mCurrentSetpoint = null;
    // The goal is the final desired state of the superstructure.
    private SuperstructureGoal mGoal;


    private SuperstructureGoal mLastValidGoal;
    private SuperstructureGoal mPreWristLevelGoal;
    private double mElevatorManual;
    private final TuckPlanner mTuckPlanner = new TuckPlanner();
    private boolean mTuck = false;
    private boolean mWantTuck = false;

    public enum ElevatorControlModes {
        OPEN_LOOP, PRISMATIC_WRIST, HEIGHT, JOGGING
    }


    private boolean mHasTarget = false;
    private boolean mOnTarget = false;

    private boolean mDisableArmAndWrist = false;
    private boolean mDisablePlanner = false;
    private boolean mDisableElevator = false;
    private double mWristHeightToMaintain = Double.NaN;
    private double mElevatorFeedforwardV = 0.0;
    private Optional<AimingParameters> mLatestAimingParameters = Optional.empty();
    private double mCorrectedRangeToTarget = 0.0;
    private boolean mEnforceAutoAimMinDistance = false;
    private double mAutoAimMinDistance = 500;

    private double mYError;

    private ElevatorControlModes mElevatorControlMode = ElevatorControlModes.HEIGHT;

    public synchronized static Superstructure getInstance() {
        if (mInstance == null) {
            mInstance = new Superstructure();
        }

        return mInstance;
    }

    private Superstructure() {}

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                synchronized (Superstructure.this) {
                    mElevatorControlMode = ElevatorControlModes.HEIGHT;
                    mWantTuck = false;
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Superstructure.this) {
                    if (mGoal == null) {
                        return;
                    }

                    updateCurrentState();
                    maybeUpdateGoalForElevator();
                    maybeUpdateGoalForDisabledArmAndWrist();
                    updateSetpointFromGoal();
                    maybeUpdateGoalFromVision(timestamp);

                    if (mCurrentSetpoint != null) {
                        followSetpoint(); // if at desired state, this should stabilize the superstructure at that state
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }

    public synchronized SuperstructureGoal getGoal() {
        return mGoal;
    }

    public synchronized SuperstructureState getCurrentState() {
        return mCurrentState;
    }

    public synchronized SuperstructureGoal getSetpoint() {
        return mCurrentSetpoint;
    }

    public synchronized void jogElevator(double delta) {
        mElevatorControlMode = ElevatorControlModes.JOGGING;
        mGoal.state.elevator = mCurrentState.elevator + delta;
        mElevatorFeedforwardV = 0.0;
    }

    public synchronized void setGoal(SuperstructureGoal goal) {
        setGoal(goal, ElevatorControlModes.HEIGHT);
    }

    public synchronized void setGoal(SuperstructureGoal goal, ElevatorControlModes elevatorControlMode) {
        System.out.println("SetGoal: " + elevatorControlMode);
        if (mGoal == null) {
            mGoal = new SuperstructureGoal(goal.state);
        }

        if (elevatorControlMode == ElevatorControlModes.PRISMATIC_WRIST
                && mElevatorControlMode != elevatorControlMode) {
            mPreWristLevelGoal = new SuperstructureGoal(mGoal.state);
        }
        if (elevatorControlMode != ElevatorControlModes.PRISMATIC_WRIST) {
            mPreWristLevelGoal = null;
        }

        mElevatorControlMode = elevatorControlMode;

        if (mElevatorControlMode != ElevatorControlModes.PRISMATIC_WRIST) {
            mGoal.state.elevator = goal.state.elevator;
        }
        mGoal.state.shoulder = goal.state.shoulder;
        mGoal.state.wrist = goal.state.wrist;
    }

    public synchronized void setElevatorManual(double control) {
        mElevatorManual = control;
    }

    private synchronized void maybeUpdateGoalForElevator() {
        if (mElevatorControlMode == ElevatorControlModes.OPEN_LOOP) {
            mGoal.state.elevator = mCurrentState.elevator;
            return;
        }

        if (mElevatorControlMode != ElevatorControlModes.PRISMATIC_WRIST) {
            return;
        }

        double elevator_height_diff = Math.sin(Math.toRadians(mArm.getActiveTrajectoryUnits())) * Constants.kArmLength;
        double elevator_height = mWristHeightToMaintain - elevator_height_diff;
        mElevatorFeedforwardV = -Math.cos(Math.toRadians(mArm.getActiveTrajectoryUnits())) * Constants.kArmLength
                * Units.degrees_to_radians(mArm.getActiveTrajectoryVelocityUnitsPerSec());

        if (elevator_height < 0 || elevator_height > Constants.kElevatorConstants.kMaxUnitsLimit) {
            elevator_height = Util.limit(elevator_height, 0.0, Constants.kElevatorConstants.kMaxUnitsLimit);
            mElevatorFeedforwardV = 0.0;
        }

        System.out.println("Updating goal for elevator.");
        mGoal.state.elevator = elevator_height;
    }

    private synchronized void maybeUpdateGoalForDisabledArmAndWrist() {
        if (mDisableArmAndWrist) {
            mGoal.state.shoulder = mCurrentState.shoulder;
            mGoal.state.wrist = mCurrentState.wrist;
        }
    }

    private synchronized void updateCurrentState() {
        mCurrentState.elevator = mElevator.getPosition();
        mCurrentState.shoulder = mArm.getAngle();
        mCurrentState.wrist = mWrist.getAngle();
    }

    private synchronized void updateSetpointFromGoal() {
        if (mLastValidGoal == null) {
            // Assume that the current state of the robot must be valid
            mLastValidGoal = new SuperstructureGoal(mCurrentState);
        }

        if (mCurrentSetpoint == null) {
            mCurrentSetpoint = new SuperstructureGoal(mCurrentState);
        }

        if (mDisablePlanner) {
            mCurrentSetpoint = new SuperstructureGoal(mGoal.state);
            mLastValidGoal.state.setFrom(mGoal.state);
            mTuck = false;
        } else {
            if (mCurrentSetpoint.isAtDesiredState(mCurrentState) || !mGoal.equals(mLastValidGoal)) {
                if (mWantTuck) {
                    // The avoid base planner is happy.  Run the tuck planner.
                    TuckPlanner.GoalWithTuck goal_with_tuck = mTuckPlanner.plan(mCurrentSetpoint, mGoal);
                    mCurrentSetpoint = goal_with_tuck.goal;
                    mTuck = goal_with_tuck.tuck;
                } else {
                    mCurrentSetpoint = mGoal;
                    mTuck = false;
                }
            }
            mLastValidGoal.state.setFrom(mGoal.state);
        }
    }

    private synchronized void followSetpoint() {
        if (mDisableElevator) {
            mElevator.setOpenLoop(0.0);
        } else if (mElevatorControlMode == ElevatorControlModes.OPEN_LOOP) {
            mElevator.setOpenLoop(mElevatorManual);
        } else if (mElevatorControlMode == ElevatorControlModes.HEIGHT) {
            mElevator.setSetpointMotionMagic(mCurrentSetpoint.state.elevator);
        } else if (mElevatorControlMode == ElevatorControlModes.PRISMATIC_WRIST ||
                mElevatorControlMode == ElevatorControlModes.JOGGING) {
            mElevator.setSetpointPositionPID(mCurrentSetpoint.state.elevator, mElevatorFeedforwardV);
        }

        if (mDisableArmAndWrist) {
            mArm.setOpenLoop(0.0);
            mWrist.setOpenLoop(0.0);
        } else {
            if (mElevatorControlMode == ElevatorControlModes.PRISMATIC_WRIST) {
                mArm.setSetpointThrust(mCurrentSetpoint.state.shoulder);
            } else {
                mArm.setSetpointMotionMagic(mCurrentSetpoint.state.shoulder);
            }
            if (mTuck) {
                double wrist = TuckPlanner.getTuckedWristAngleForArm(mCurrentState, mCurrentSetpoint);
                double wrist_feedforward = TuckPlanner.getFeedforwardWristVelocity(mCurrentState.shoulder, mArm.getVelocity());
                mWrist.setSetpointPositionPID(wrist, wrist_feedforward);
            } else {
                mWrist.setSetpointMotionMagic(mCurrentSetpoint.state.wrist);
            }
        }
    }

    private synchronized void maybeUpdateGoalFromVision(double timestamp) {
        boolean useHighTarget = mRobotState.useHighTarget();
        mLatestAimingParameters = mRobotState.getAimingParameters(useHighTarget, -1, Constants.kMaxGoalTrackAge);
        if (mLatestAimingParameters.isPresent()) {
            final double kLookaheadTime = 0.7;
            Pose2d robot_to_predicted_robot = mRobotState.getLatestFieldToVehicle().getValue().inverse()
                    .transformBy(mRobotState.getPredictedFieldToVehicle(kLookaheadTime));
            Pose2d predicted_robot_to_goal = robot_to_predicted_robot.inverse()
                    .transformBy(mLatestAimingParameters.get().getRobotToGoal());

            mYError = predicted_robot_to_goal.getTranslation().y();

            mCorrectedRangeToTarget = predicted_robot_to_goal.getTranslation().norm();

            // Don't aim if not in min distance
            if (mEnforceAutoAimMinDistance && mCorrectedRangeToTarget > mAutoAimMinDistance) {
                return;
            }

            Rotation2d heading_error = mRobotState.getVehicleToTurret(timestamp).getRotation().inverse()
                    .rotateBy(mLatestAimingParameters.get().getRobotToGoalRotation());

            mHasTarget = true;

            mOnTarget = Util.epsilonEquals(heading_error.getDegrees(), 0.0, 3.0);
        } else {
            mHasTarget = false;
            mOnTarget = false;
        }
    }

    public synchronized boolean isOnTarget() {
        return mOnTarget;
    }

    public synchronized double getCorrectedRangeToTarget() {
        return mCorrectedRangeToTarget;
    }

    public synchronized void setPlannerDisabled() {
        mDisablePlanner = true;
    }

    public synchronized void setPlannerEnabled() {
        mDisablePlanner = false;
    }

    public synchronized double getYError() {
        return mYError;
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public synchronized void outputTelemetry() {
        SmartDashboard.putString("elevator control mode", mElevatorControlMode.toString());
        SmartDashboard.putBoolean("has target", mHasTarget);
    }

    public synchronized boolean hasTarget() {
        return mHasTarget;
    }

    public synchronized void setDisableArmAndWrist(boolean disableArmAndWrist) {
        mDisableArmAndWrist = disableArmAndWrist;
    }

    public synchronized void setUseElevatorManual(boolean wantsElevatorManual) {
        mElevatorControlMode = wantsElevatorManual ? ElevatorControlModes.OPEN_LOOP : ElevatorControlModes.HEIGHT;
    }

    public synchronized SuperstructureGoal getPreWristLevelGoal() {
        return mPreWristLevelGoal;
    }

    public synchronized void overridePreWristLevelGoal(SuperstructureGoal goal) {
        mPreWristLevelGoal = goal;
    }

    public synchronized boolean isAtDesiredState() {
        return mCurrentState != null && mGoal != null && mGoal.isAtDesiredState(mCurrentState);
    }

    public synchronized void setHeightForWristLevel(double wristHeight) {
        mWristHeightToMaintain = wristHeight;
    }

    public synchronized ElevatorControlModes getElevatorControlMode() {
        return mElevatorControlMode;
    }

    public synchronized double getHeightForWristLevel() {
        return mWristHeightToMaintain;
    }

    public synchronized void setWantTuck(boolean wantTuck) {
        mWantTuck = wantTuck;
    }

    public synchronized void setDisableElevator(boolean disable) {
        mDisableElevator = disable;
    }
}
