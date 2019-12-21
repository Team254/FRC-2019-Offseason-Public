package com.team254.frc2019;

import com.team254.frc2019.auto.AutoModeExecutor;
import com.team254.frc2019.auto.modes.AutoModeBase;
import com.team254.frc2019.controlboard.ControlBoard;
import com.team254.frc2019.controlboard.IControlBoard;
import com.team254.frc2019.loops.Looper;
import com.team254.frc2019.statemachines.EndEffectorStateMachine;
import com.team254.frc2019.statemachines.SuctionClimbingStateMachine;
import com.team254.frc2019.statemachines.SuperstructureCommands;
import com.team254.frc2019.subsystems.*;
import com.team254.lib.control.SwerveHeadingController;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.*;
import com.team254.lib.wpilib.TimedRobot;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

public class Robot extends TimedRobot {
    private final Looper mEnabledLooper = new Looper();
    private final Looper mDisabledLooper = new Looper();

    private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();

    private final RobotState mRobotState = RobotState.getInstance();
    private final RobotStateEstimator mRobotStateEstimator = RobotStateEstimator.getInstance();
    private final Drive mDrive = Drive.getInstance();
    private final Elevator mElevator = Elevator.getInstance();
    private final Arm mArm = Arm.getInstance();
    private final Wrist mWrist = Wrist.getInstance();
    private final EndEffector mEndEffector = EndEffector.getInstance();
    private final Vacuum mVacuum = Vacuum.getInstance();
    private final CarriageCanifier mCarriageCanifier = CarriageCanifier.getInstance();
    private final Infrastructure mInfrastructure = Infrastructure.getInstance();
    private final Superstructure mSuperstructure = Superstructure.getInstance();
    private final Kickstand mKickstand = Kickstand.getInstance();
    private final LED mLED = LED.getInstance();
    private final LimelightManager mLLManager = LimelightManager.getInstance();
    private SuctionClimbingStateMachine mSuctionStateMachine = new SuctionClimbingStateMachine();

    private final SwerveHeadingController mSwerveHeadingController = SwerveHeadingController.getInstance();

    private IControlBoard mControlBoard = ControlBoard.getInstance();

    private TimeDelayedBoolean mHangModeEnablePressed = new TimeDelayedBoolean();
    private TimeDelayedBoolean mHangModeLowEnablePressed = new TimeDelayedBoolean();
    private boolean mInHangMode;
    private boolean mIntakeButtonPressed = false;
    private boolean mHangModeReleased = true;

    private MultiTrigger mDiskIntakeTrigger = new MultiTrigger(.4);
    private MultiTrigger mBallIntakeTrigger = new MultiTrigger(.4);

    private boolean mHasBeenEnabled = false;
    private DigitalInput mResetButton = new DigitalInput(Constants.kResetButtonChannel);

    private double mLastShootPressedTime = -1.0;
    private double mOffsetOverride = -1.0;

    private LatchedBoolean mShootPressed = new LatchedBoolean();
    private boolean mStickyShoot;

    private LatchedBoolean mThrustReleased = new LatchedBoolean();
    private LatchedBoolean mThrustPressed = new LatchedBoolean();
    private double mLastThrustPressedTime = -1.0;
    private double mLastThrustShotTime = Double.NaN;

    private AutoModeSelector mAutoModeSelector = new AutoModeSelector();
    private AutoModeExecutor mAutoModeExecutor;
    private boolean mDriveByCameraInAuto = false;

    Robot() {
        CrashTracker.logRobotConstruction();
    }

    @Override
    public void robotInit() {
        try {
            CrashTracker.logRobotInit();

            mSubsystemManager.setSubsystems(new Subsystem[] {mRobotStateEstimator, mDrive, mArm, mElevator, mWrist,
                            mCarriageCanifier, mInfrastructure, mEndEffector, mVacuum, mKickstand, mLED, mSuperstructure, mLLManager},
                    mDrive.getSwerveModules());

            System.out.println("ZEROING ROBOT");
            mElevator.zeroSensors();
            mArm.zeroSensors();
            mWrist.zeroSensors();
            mCarriageCanifier.zeroSensors();

            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            mSubsystemManager.registerDisabledLoops(mDisabledLooper);

            mLED.registerEnabledLoops(mDisabledLooper);
            mLED.registerEnabledLoops(mEnabledLooper);

            mRobotState.reset(Timer.getFPGATimestamp(), Pose2d.identity());

            mAutoModeSelector.updateModeCreator();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledInit() {
        try {
            CrashTracker.logDisabledInit();
            mEnabledLooper.stop();

            // Reset all auto mode state.
            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }
            mAutoModeSelector.reset();
            mAutoModeSelector.updateModeCreator();
            mAutoModeExecutor = new AutoModeExecutor();

            mDrive.zeroSensors();

            mDisabledLooper.start();

            mInfrastructure.setIsManualControl(false);
            if (!mInHangMode) {
                mLED.setWantedAction(LED.WantedAction.DISPLAY_ZEROING);
            } else {
                mLED.setWantedAction(LED.WantedAction.DISPLAY_HANG);
            }
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousInit() {
        try {
            mHasBeenEnabled = true;

            CrashTracker.logAutoInit();
            mDisabledLooper.stop();

            mRobotState.reset(Timer.getFPGATimestamp(), Pose2d.identity());
            mDrive.zeroSensors();

            System.out.println("Auto init - " + mDriveByCameraInAuto);
            if (!mDriveByCameraInAuto) {
                mAutoModeExecutor.start();
            }

            mEnabledLooper.start();

            mLLManager.setUseTopLimelight(true);
            mLLManager.setPipeline(Limelight.kDefaultPipeline);

            mLED.setWantedAction(LED.WantedAction.DISPLAY_INTAKE);
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void teleopInit() {
        try {
            mLED.setWantedAction(LED.WantedAction.DISPLAY_INTAKE);
            mHasBeenEnabled = true;

            CrashTracker.logTeleopInit();
            mDisabledLooper.stop();
            mEnabledLooper.start();

            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }

            // Force true on first iteration of teleop periodic
            shouldChangeAzimuthSetpoint.update(false);

            mSuperstructure.setDisableElevator(false);
            mSuperstructure.setUseElevatorManual(false);
            mKickstand.setDisengaged();
            mKickstand.setRachetDisengaged();

            mLLManager.setUseTopLimelight(true);
            mLLManager.setPipeline(Limelight.kDefaultPipeline);

            mSuctionStateMachine = new SuctionClimbingStateMachine();
            mInfrastructure.setIsManualControl(true);
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testInit() {
        try {
            CrashTracker.logTestInit();
            System.out.println("Starting check systems.");

            mDisabledLooper.stop();
            mEnabledLooper.stop();

            if (mSubsystemManager.checkSubsystems()) {
                System.out.println("ALL SYSTEMS PASSED");
            } else {
                System.out.println("CHECK ABOVE OUTPUT SOME SYSTEMS FAILED!!!");
            }
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    LatchedBoolean shouldChangeAzimuthSetpoint = new LatchedBoolean();

    @Override
    public void robotPeriodic() {
        try {
            mSubsystemManager.outputToSmartDashboard();
            mRobotState.outputToSmartDashboard();
            mSuctionStateMachine.outputToSmartDashboard();
            mAutoModeSelector.outputToSmartDashboard();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledPeriodic() {
        try {
            if (!mResetButton.get() && !mHasBeenEnabled) {
                System.out.println("Zeroing Robot!!!!");
                mLED.updateZeroed();

                mElevator.zeroSensors();
                mArm.zeroSensors();
                mWrist.zeroSensors();
                mCarriageCanifier.zeroSensors();
            }

            // Update auto modes
            mAutoModeSelector.updateModeCreator();

            mDrive.setHeading(Rotation2d.identity());
            mSwerveHeadingController.setHeadingControllerState(SwerveHeadingController.HeadingControllerState.OFF);

            Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode();
            mDriveByCameraInAuto = mAutoModeSelector.isDriveByCamera();
            if (autoMode.isPresent() && autoMode.get() != mAutoModeExecutor.getAutoMode()) {
                System.out.println("Set auto mode to: " + autoMode.get().getClass().toString());
                mAutoModeExecutor.setAutoMode(autoMode.get());
            }
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousPeriodic() {
        try {
            if (mDriveByCameraInAuto) {
                manualControl();
            }
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    TimeDelayedBoolean mShouldMaintainAzimuth = new TimeDelayedBoolean();
    SynchronousPIDF mAlignPID = new SynchronousPIDF(0.045, 0, 0.001, 0);

    @Override
    public void teleopPeriodic() {
        try {
            manualControl();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testPeriodic() {}

    public void manualControl() {
        double timestamp = Timer.getFPGATimestamp();
        boolean rumble = false;
        GamePiece gamePiece = mEndEffector.getObservedGamePiece();

        boolean wantShoot = mControlBoard.getShoot();
        boolean shotJustPushed = mShootPressed.update(wantShoot);
        if (shotJustPushed) {
            mLastShootPressedTime = Timer.getFPGATimestamp();
        }

        boolean hangModePressed =
                mHangModeEnablePressed.update(mControlBoard.getToggleHangMode(), 0.250);
        boolean hangModeLowPressed =
                mHangModeLowEnablePressed.update(mControlBoard.getToggleHangModeLow(), 0.250);

        if ((hangModeLowPressed && hangModePressed) && !mInHangMode && mHangModeReleased) {
            System.out.println("Entering hang mode for low: " + hangModeLowPressed + " high: " + hangModePressed);
            mInHangMode = true;
            mLED.setWantedAction(LED.WantedAction.DISPLAY_HANG);
            mEndEffector.setForcedJawState(EndEffectorStateMachine.EndEffectorState.JawState.CLOSED);
            mElevator.setCanHome(false);
            mHangModeReleased = false;
        } else if ((hangModeLowPressed && hangModePressed) && mInHangMode && mHangModeReleased) {
            System.out.println("Exiting hang mode!");
            mInHangMode = false;
            mLED.setWantedAction(LED.WantedAction.DISPLAY_INTAKE);
            mEndEffector.setForcedJawState(null);
            mElevator.setCanHome(true);
            mSuctionStateMachine.reset();
            mSuctionStateMachine = new SuctionClimbingStateMachine();
            mHangModeReleased = false;
        }

        if (!hangModeLowPressed && !hangModePressed) {
            mHangModeReleased = true;
        }

        // commands
        mDiskIntakeTrigger.update(mControlBoard.getPickupDiskWall());
        mBallIntakeTrigger.update(mControlBoard.getPickupBallGround());

        if (mInHangMode) {
            mSuctionStateMachine.handle(Timer.getFPGATimestamp(), mControlBoard.getScorePresetLow(), mControlBoard.getThrust());
        } else {
            // End Effector Jog
            double jogZ = mControlBoard.getJoggingZ();
            boolean wants_thrust = mControlBoard.getThrust();

            boolean thrust_just_pressed = mThrustPressed.update(wants_thrust);
            if (thrust_just_pressed || wantShoot) {
                if (SuperstructureCommands.isInLowPosition() &&
                        gamePiece == GamePiece.BALL) {
                    if (mSuperstructure.isOnTarget() || wantShoot) {
                        SuperstructureCommands.goToScoreLow();
                    } else {
                        // Try again!
                        mThrustPressed.update(false);
                    }
                }
            }
            if (thrust_just_pressed) {
                mLastThrustPressedTime = timestamp;
            }

            if (wants_thrust) {
                double distanceLeft = SuperstructureCommands.goToAutoScore(mOffsetOverride);
                SmartDashboard.putNumber("distance left", distanceLeft);


                if (mSuperstructure.isOnTarget() || true) {
                    if (mEndEffector.getObservedGamePiece() == GamePiece.BALL &&
                            !SuperstructureCommands.isInLowPosition() &&
                            distanceLeft < 1.0) {
                        mStickyShoot = true;
                    }
                    if (mSuperstructure.isAtDesiredState()) {
                        wantShoot = true;
                        mStickyShoot = true;
                    }
                    if (timestamp - mLastThrustPressedTime > 0.75) {
                        mStickyShoot = true;
                    }
                }

                if (mStickyShoot) {
                    wantShoot = true;
                    mLastThrustShotTime = timestamp;
                }
            } else {
                mStickyShoot = false;
                if (!Util.epsilonEquals(Math.abs(jogZ), 0.0) && !mElevator.isHoming()) {
                    SuperstructureCommands.jogEndEffector(0, jogZ * Constants.kJogElevatorScaler);
                }
                final double kLatchShootingTime = 0.75;
                if (!Double.isNaN(mLastThrustShotTime) && timestamp - mLastThrustShotTime < kLatchShootingTime) {
                    wantShoot = true;
                }
            }

            if (mElevator.isHoming()) {
                mSuperstructure.setElevatorManual(jogZ * 0.5);
                mSuperstructure.setUseElevatorManual(true);
            }

            mSuperstructure.setWantTuck(mControlBoard.getTuck());

            if (mBallIntakeTrigger.isPressed() && mControlBoard.getScorePresetLow()) {
                SuperstructureCommands.goToPickupBallFromGround();
                mOffsetOverride = -1.0;
            } else if (mControlBoard.getPickupBallGround()) {
                if (mEndEffector.getEndEffectorSystemState() == EndEffectorStateMachine.SystemState.HAVE_CARGO && mElevator.getPosition() < 10.0) {
                    SuperstructureCommands.goToStowedWithBall();
                }
                mEndEffector.updateObservedGamePiece(GamePiece.BALL);
                mOffsetOverride = -1.0;
            } else if (mControlBoard.getScorePresetLow() && !mControlBoard.getPickupDiskWall()) {
                if (gamePiece == GamePiece.BALL) {
                    SuperstructureCommands.goToPreScoreLow();
                } else {
                    SuperstructureCommands.goToScoreLow();
                }
                mLLManager.setUseTopLimelight(true);
                mLLManager.updatePipeline(gamePiece);
                mRobotState.resetVision();
                mOffsetOverride = -1.0;
            } else if (mControlBoard.getScorePresetMiddle()) {
                SuperstructureCommands.goToScoreMiddle();
                mLLManager.setUseTopLimelight(false);
                mLLManager.updatePipeline(gamePiece);
                mRobotState.resetVision();
                mOffsetOverride = -1.0;
            } else if (mControlBoard.getScorePresetHigh()) {
                SuperstructureCommands.goToScoreHigh();
                mLLManager.updatePipeline(gamePiece);
                mLLManager.setUseTopLimelight(false);
                mRobotState.resetVision();
                mOffsetOverride = -1.0;
            } else if (mControlBoard.getScorePresetCargo()) {
                SuperstructureCommands.goToScoreCargo();
                mLLManager.setPipeline(Limelight.kDefaultPipeline);
                mLLManager.setUseTopLimelight(false);
                mRobotState.resetVision();
                mOffsetOverride = -1.0;
            } else if (mControlBoard.getPickupDiskWall()) {
                if (mControlBoard.getScorePresetLow()) {
                    SuperstructureCommands.goToPickupDiskFromWallFar();
                } else {
                    SuperstructureCommands.goToPickupDiskFromWall();
                }
                mLLManager.setUseTopLimelight(true);
                mLLManager.updatePipeline(gamePiece);
                mOffsetOverride = -1.0;
            }

            // end effector state
            if (wantShoot && (!SuperstructureCommands.isInLowPosition() ||
                    mSuperstructure.isAtDesiredState() ||
                    (Timer.getFPGATimestamp() - mLastShootPressedTime > 1.0))) {
                mEndEffector.setWantedAction(EndEffectorStateMachine.WantedAction.EXHAUST);
                mIntakeButtonPressed = true;
            } else if (mControlBoard.getPickupDiskWall()) {
                mEndEffector.setWantedAction(EndEffectorStateMachine.WantedAction.INTAKE_DISC);
                if (mEndEffector.hasDisk()) {
                    rumble = true;
                }
                mIntakeButtonPressed = true;
            } else if (mControlBoard.getPickupBallGround()) {
                mEndEffector.setWantedAction(EndEffectorStateMachine.WantedAction.INTAKE_CARGO);
                if (mCarriageCanifier.hasBall()) {
                    rumble = true;
                }
                mIntakeButtonPressed = true;
            } else if (mIntakeButtonPressed) {
                mEndEffector.setWantedAction(EndEffectorStateMachine.WantedAction.IDLE);
            }

            if (mThrustReleased.update(!mControlBoard.getThrust())) {
                if (SuperstructureCommands.isInLowPosition() &&
                        gamePiece == GamePiece.BALL) {
                    mSuperstructure.overridePreWristLevelGoal(SuperstructureCommands.PreScoreBallLowGoal);
                }
                SuperstructureCommands.goToPreAutoThrust();
            }

            mControlBoard.setRumble(rumble);
        }

        // drive
        boolean maintainAzimuth = mShouldMaintainAzimuth.update(mControlBoard.getRotation() == 0, 0.2);
        boolean changeAzimuthSetpoint = shouldChangeAzimuthSetpoint.update(maintainAzimuth);

        if (mControlBoard.getHorizontalAlign() && mSuperstructure.hasTarget()) {
            double yError = mSuperstructure.getYError();
            SmartDashboard.putNumber("Horizontal error", yError);
            mAlignPID.setSetpoint(0);
            double yOut = mAlignPID.calculate(yError);

            mDrive.setTeleopInputs(mControlBoard.getThrottle(), -yOut, mSwerveHeadingController.update(),
                    false, false, true);
        } else {

            if (mControlBoard.getDPad() != -1) {
                mSwerveHeadingController.setHeadingControllerState(SwerveHeadingController.HeadingControllerState.SNAP);
                double heading_goal = mControlBoard.getDPad();
                SmartDashboard.putNumber("Heading Goal", heading_goal);
                mSwerveHeadingController.setGoal(heading_goal);
            } else {
                if (!maintainAzimuth) {
                    mSwerveHeadingController.setHeadingControllerState(SwerveHeadingController.HeadingControllerState.OFF);
                } else if ((mSwerveHeadingController
                        .getHeadingControllerState() == SwerveHeadingController.HeadingControllerState.SNAP
                        && mSwerveHeadingController.isAtGoal()) || changeAzimuthSetpoint) {
                    mSwerveHeadingController
                            .setHeadingControllerState(SwerveHeadingController.HeadingControllerState.MAINTAIN);
                    mSwerveHeadingController.setGoal(mDrive.getHeading().getDegrees());
                }
            }

            if (mSwerveHeadingController.getHeadingControllerState() != SwerveHeadingController.HeadingControllerState.OFF) {
                mDrive.setTeleopInputs(mControlBoard.getThrottle(), mControlBoard.getStrafe(), mSwerveHeadingController.update(),
                        mControlBoard.getDriveLowPower(), mControlBoard.getFieldRelative(), true);
            } else {
                mDrive.setTeleopInputs(mControlBoard.getThrottle(), mControlBoard.getStrafe(), mControlBoard.getRotation(),
                        mControlBoard.getDriveLowPower(), mControlBoard.getFieldRelative(), false);
            }

        }
    }
}