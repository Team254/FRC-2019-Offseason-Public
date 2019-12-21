package com.team254.frc2019;

import com.team254.frc2019.subsystems.Limelight.LimelightConstants;
import com.team254.frc2019.subsystems.ServoMotorSubsystem.ServoMotorSubsystemConstants;
import com.team254.frc2019.subsystems.ServoMotorSubsystem.TalonSRXConstants;
import com.team254.frc2019.subsystems.SwerveModule.SwerveModuleConstants;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;

/**
 * A list of constants used by the rest of the robot code. This includes physics
 * constants as well as constants determined through calibration.
 */
public class Constants {
    public static final double kLooperDt = 0.01;
    public static final double kRobotPeriodicDt = 0.02;

    public static final int kCANTimeoutMs = 10; // use for important on the fly updates
    public static final int kLongCANTimeoutMs = 100; // use for constructors

    // controls
    public static final int kDriveGamepadPort = 0;
    public static final int kButtonGamepadPort = 2;
    public static final int kMainThrottleJoystickPort = 0;
    public static final int kMainTurnJoystickPort = 1;
    public static final double kJoystickThreshold = 0.5;

    // drivebase
    public static final double kDriveWheelbase = 28.5;
    public static final double kDriveTrackwidth = 28;
    public static final double kBumperHeight = 6.6 + 2.0; // inches to ground + 2 in buffer (TODO: double check)

    // swerve modules
    // zero - bezel to the left
    public static final SwerveModuleConstants kFrontRightModuleConstants = new SwerveModuleConstants();

    /*
     * FL 4196.0 - 45.0 * 4096.0 / 360.0 = 3,684 + 2048 //invert FR 1,327 + 45 *
     * 4096.0 / 360.0 = 1839 BL 5120.0 - 135 * 4096 / 360 = 3584 + 2048 //invert BR
     * 2060.0 + 135 * 4096 / 360 = 3596
     *
     */

    static {
        kFrontRightModuleConstants.kName = "Front Right";
        kFrontRightModuleConstants.kDriveTalonId = 1;
        kFrontRightModuleConstants.kAzimuthTalonId = 2;
        kFrontRightModuleConstants.kAzimuthEncoderHomeOffset = 883.0;
        /* ... */
    }

    public static final SwerveModuleConstants kFrontLeftModuleConstants = new SwerveModuleConstants();

    static {
        kFrontLeftModuleConstants.kName = "Front Left";
        kFrontLeftModuleConstants.kDriveTalonId = 3;
        kFrontLeftModuleConstants.kAzimuthTalonId = 4;
        kFrontLeftModuleConstants.kAzimuthEncoderHomeOffset = 1683.0;
        /* ... */
    }

    public static final SwerveModuleConstants kBackLeftModuleConstants = new SwerveModuleConstants();

    static {
        kBackLeftModuleConstants.kName = "Back Left";
        kBackLeftModuleConstants.kDriveTalonId = 5;
        kBackLeftModuleConstants.kAzimuthTalonId = 6;
        kBackLeftModuleConstants.kAzimuthEncoderHomeOffset = 3451.0;
        /* ... */
    }

    public static final SwerveModuleConstants kBackRightModuleConstants = new SwerveModuleConstants();

    static {
        kBackRightModuleConstants.kName = "Back Right";
        kBackRightModuleConstants.kDriveTalonId = 7;
        kBackRightModuleConstants.kAzimuthTalonId = 8;
        kBackRightModuleConstants.kAzimuthEncoderHomeOffset = -327.0;
        /* ... */
    }

    // Swerve Heading Controller
    public static final double kSwerveHeadingControllerErrorTolerance = 1.0; // degrees

    // good for snapping (dpad)
    public static final double kSnapSwerveHeadingKp = 0.031;
    public static final double kSnapSwerveHeadingKi = 0.0;
    public static final double kSnapSwerveHeadingKd = 0.003;

    // good for maintaining heading
    public static final double kMaintainSwerveHeadingKp = 0.020;
    public static final double kMaintainSwerveHeadingKi = 0.0;
    public static final double kMaintainSwerveHeadingKd = 0.002;

    // gyro
    public static final int kPigeonId = 15;

    // limelight
    public static final double kHorizontalFOV = 59.6; // degrees
    public static final double kVerticalFOV = 49.7; // degrees
    public static final double kVPW = 2.0 * Math.tan(Math.toRadians(kHorizontalFOV / 2.0));
    public static final double kVPH = 2.0 * Math.tan(Math.toRadians(kVerticalFOV / 2.0));
    public static final double kImageCaptureLatency = 11.0 / 1000.0; // seconds

    public static final double kMinStability = 0.5;
    public static final int kPortPipeline = 0;
    public static final int kBallPipeline = 2;
    public static final double kPortTargetHeight = 39.125;
    public static final double kHatchTargetHeight = 31.5;

    public static final double kTurretToArmOffset = -2.5;  // in
    public static final double kWristToTremorsEnd = 15.75;  // in

    public static final double kMaxTrackerDistance = 9.0;
    public static final double kMaxGoalTrackAge = 2.5;
    public static final double kMaxGoalTrackAgeNotTracking = 0.1;
    public static final double kMaxGoalTrackSmoothingTime = 0.5;
    public static final double kTrackStabilityWeight = 0.0;
    public static final double kTrackAgeWeight = 10.0;
    public static final double kTrackSwitchingWeight = 100.0;
    public static final double kCameraFrameRate = 90.0;

    // elevator
    public static final ServoMotorSubsystemConstants kElevatorConstants = new ServoMotorSubsystemConstants();
    static {
        kElevatorConstants.kName = "Elevator";

        kElevatorConstants.kMasterConstants.id = 9;
        kElevatorConstants.kMasterConstants.invert_motor = false;
        kElevatorConstants.kMasterConstants.invert_sensor_phase = false;
        kElevatorConstants.kSlaveConstants = new TalonSRXConstants[2];

        kElevatorConstants.kSlaveConstants[0] = new TalonSRXConstants();
        kElevatorConstants.kSlaveConstants[1] = new TalonSRXConstants();

        kElevatorConstants.kSlaveConstants[0].id = 10;
        kElevatorConstants.kSlaveConstants[0].invert_motor = false;
        kElevatorConstants.kSlaveConstants[1].id = 11;
        kElevatorConstants.kSlaveConstants[1].invert_motor = false;

        // Unit == Inches
        kElevatorConstants.kHomePosition = 10.25; // Inches off ground
        kElevatorConstants.kTicksPerUnitDistance = 4096.0 / (1.75 * Math.PI);
        kElevatorConstants.kKp = 0.5;
        kElevatorConstants.kKi = 0;
        kElevatorConstants.kKd = 10;
        kElevatorConstants.kKf = .248;
        kElevatorConstants.kKa = 0.0;
        kElevatorConstants.kMaxIntegralAccumulator = 0;
        kElevatorConstants.kIZone = 0; // Ticks
        kElevatorConstants.kDeadband = 0; // Ticks

        kElevatorConstants.kPositionKp = 0.5;
        kElevatorConstants.kPositionKi = 0;
        kElevatorConstants.kPositionKd = 10;
        kElevatorConstants.kPositionKf = 0;
        kElevatorConstants.kPositionMaxIntegralAccumulator = 0;
        kElevatorConstants.kPositionIZone = 0; // Ticks
        kElevatorConstants.kPositionDeadband = 0; // Ticks

        kElevatorConstants.kMaxUnitsLimit = 31.1; // inches
        kElevatorConstants.kMinUnitsLimit = 0.0; // inches

        kElevatorConstants.kCruiseVelocity = 4000; // Ticks / 100ms
        kElevatorConstants.kAcceleration = 8000; // Ticks / 100ms / s
        kElevatorConstants.kRampRate = 0.005; // s
        kElevatorConstants.kContinuousCurrentLimit = 35; // amps
        kElevatorConstants.kPeakCurrentLimit = 40; // amps
        kElevatorConstants.kPeakCurrentDuration = 10; // milliseconds

    }
    public static final double kJogElevatorScaler = 0.4;
    public static final double kElevatorHeightToFloor = 23.337; // (in) height of arm joint to floor when elevator is at 0 pose

    // arm
    public static final ServoMotorSubsystemConstants kArmConstants = new ServoMotorSubsystemConstants();
    static {
        kArmConstants.kName = "Arm";

        kArmConstants.kMasterConstants.id = 12;
        kArmConstants.kMasterConstants.invert_motor = false;
        kArmConstants.kMasterConstants.invert_sensor_phase = false;

        // Unit == Degrees
        kArmConstants.kHomePosition = -90.0; // Degrees
        kArmConstants.kTicksPerUnitDistance = (4096.0 * 3.0) / 360.0;
        kArmConstants.kKp = 1.0;
        kArmConstants.kKi = 0;
        kArmConstants.kKd = 50.0;
        kArmConstants.kKf = 1.15;
        kArmConstants.kKa = 0.0;
        kArmConstants.kMaxIntegralAccumulator = 0;
        kArmConstants.kIZone = 0; // Ticks
        kArmConstants.kDeadband = 0; // Ticks

        kArmConstants.kPositionKp = 1.0;
        kArmConstants.kKi = 0;
        kArmConstants.kPositionKd = 50.0;
        kArmConstants.kPositionKf = 0.0;
        kArmConstants.kPositionMaxIntegralAccumulator = 0;
        kArmConstants.kPositionIZone = 0; // Ticks
        kArmConstants.kPositionDeadband = 0; // Ticks

        kArmConstants.kMinUnitsLimit = -90.0;
        kArmConstants.kMaxUnitsLimit = 90.0;

        kArmConstants.kCruiseVelocity = 700; // Ticks / 100ms
        kArmConstants.kAcceleration = 2000; // Ticks / 100ms / s
        kArmConstants.kRampRate = 0.001; // s
        kArmConstants.kContinuousCurrentLimit = 40; // amps
        kArmConstants.kPeakCurrentLimit = 60; // amps
        kArmConstants.kPeakCurrentDuration = 200; // milliseconds
    }
    public static final int kArmCruiseVelocityForThrust = 350; // Ticks / 100ms
    public static final double kArmLength = 21.0; // inches

    // wrist
    public static final ServoMotorSubsystemConstants kWristConstants = new ServoMotorSubsystemConstants();
    static {
        kWristConstants.kName = "Wrist";

        kWristConstants.kMasterConstants.id = 13;
        kWristConstants.kMasterConstants.invert_motor = false;
        kWristConstants.kMasterConstants.invert_sensor_phase = true;

        // Unit == Degrees
        kWristConstants.kHomePosition = 0.0; // Degrees
        kWristConstants.kTicksPerUnitDistance = (4096.0 * 3.0) / 360.0;
        kWristConstants.kKp = 2.25;
        kWristConstants.kKi = 0;
        kWristConstants.kKd = 200;
        kWristConstants.kKf = 0.6;
        kWristConstants.kMaxIntegralAccumulator = 0;
        kWristConstants.kIZone = 0; // Ticks
        kWristConstants.kDeadband = 0; // Ticks

        kWristConstants.kPositionKp = 2.25;
        kWristConstants.kPositionKi = 0;
        kWristConstants.kPositionKd = 200;
        kWristConstants.kPositionKf = 0.0;
        kWristConstants.kPositionMaxIntegralAccumulator = 0;
        kWristConstants.kPositionIZone = 0; // Ticks
        kWristConstants.kPositionDeadband = 0; // Ticks

        kWristConstants.kMinUnitsLimit = -135.0;
        kWristConstants.kMaxUnitsLimit = 45.0;

        kWristConstants.kCruiseVelocity = 800; // Ticks / 100ms
        kWristConstants.kAcceleration = 1600; // Ticks / 100ms / s
        kWristConstants.kRampRate = 0.005; // s
        kWristConstants.kContinuousCurrentLimit = 40; // amps
        kWristConstants.kPeakCurrentLimit = 60; // amps
        kWristConstants.kPeakCurrentDuration = 200; // milliseconds
    }
    public static final double kWristToBottomEndEffectorLength = 15.91; // Length (in) from wrist joint to bottom of end effector
    public static final double kEndEffectorBottomAngle = 7.75; // Angle (°) from wrist joint to bottom of end effector when wrist is at 0°

    // end effector
    public static final int kBallIntakeMasterId = 16;
    public static final int kTremorsMasterId = 15;

    // vacuum pump
    public static final int kVacuumMasterId = 14;

    // reset button
    public static final int kResetButtonChannel = 4;

    // canifier
    public static final int kCanifierArmId = 17;
    public static final int kCanifierWristId = 18;

    // pigeon imu
    public static final int kPigeonIMUId = 19;

    // solenoids
    public static final int kPCMId = 1;
    public static final int kShifterSolenoidId = 7;
    public static final int kBallIntakeJawId = 6;
    public static final int kKickstandForwardId = 1; // deploys
    public static final int kKickstandReverseId = 0; // retracts
    public static final int kRatchetForwardId = 3; // deploys
    public static final int kRatchetReverseId = 2; // retracts

    // Top limelight
    public static final LimelightConstants kTopLimelightConstants = new LimelightConstants();
    static {
        kTopLimelightConstants.kName = "Top Limelight";
        kTopLimelightConstants.kTableName = "limelight-top";
        kTopLimelightConstants.kHeight = 44.047;  // inches
        kTopLimelightConstants.kTurretToLens = new Pose2d(new Translation2d(-7.685 - 0.25, 0.0), Rotation2d.fromDegrees(0.0));
        kTopLimelightConstants.kHorizontalPlaneToLens = Rotation2d.fromDegrees(-24.0);
    }

    // Bottom limelight
    public static final LimelightConstants kBottomLimelightConstants = new LimelightConstants();
    static {
        kBottomLimelightConstants.kName = "Bottom Limelight";
        kBottomLimelightConstants.kTableName = "limelight-bottom";
        kBottomLimelightConstants.kHeight = 7.221;  // inches
        kBottomLimelightConstants.kTurretToLens = new Pose2d(new Translation2d(-1.293 - 0.25, 2.556), Rotation2d.fromDegrees(2.0));
        kBottomLimelightConstants.kHorizontalPlaneToLens = Rotation2d.fromDegrees(47.5);
    }

    public static final double kMaxTopLimelightHeight = 16.0;

    /**
     * @return the MAC address of the robot
     */
    public static String getMACAddress() {
        try {
            Enumeration<NetworkInterface> nwInterface = NetworkInterface.getNetworkInterfaces();
            StringBuilder ret = new StringBuilder();
            while (nwInterface.hasMoreElements()) {
                NetworkInterface nis = nwInterface.nextElement();
                if (nis != null) {
                    byte[] mac = nis.getHardwareAddress();
                    if (mac != null) {
                        for (int i = 0; i < mac.length; i++) {
                            ret.append(String.format("%02X%s", mac[i], (i < mac.length - 1) ? "-" : ""));
                        }
                        return ret.toString();
                    } else {
                        System.out.println("Address doesn't exist or is not accessible");
                    }
                } else {
                    System.out.println("Network Interface for the specified address is not found.");
                }
            }
        } catch (SocketException | NullPointerException e) {
            e.printStackTrace();
        }

        return "";
    }
}
