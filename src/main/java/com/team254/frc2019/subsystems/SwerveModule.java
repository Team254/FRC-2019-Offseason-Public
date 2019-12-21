package com.team254.frc2019.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team254.frc2019.Constants;
import com.team254.frc2019.loops.ILooper;
import com.team254.frc2019.loops.Loop;
import com.team254.lib.drivers.TalonSRXFactory;
import com.team254.lib.drivers.TalonSRXUtil;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule extends Subsystem {
    public static class PeriodicIO {
        // INPUTS
        public double drive_encoder_ticks;
        public double azimuth_encoder_ticks; // actual position of module in encoder units, adjusted for home offset
        public int position_ticks;
        public int distance;
        public int velocity_ticks_per_100ms;

        // OUTPUTS
        public double drive_demand;
        public double azimuth_demand; // actual desired demand in encoder units, not adjusted for home offset
    }

    public enum ControlState {
        OPEN_LOOP
    }

    public static class SwerveModuleConstants {
        public String kName = "Name";
        public int kDriveTalonId = -1;
        public int kAzimuthTalonId = -1;

        // general azimuth
        public boolean kInvertAzimuth = false;
        public boolean kInvertAzimuthSensorPhase = false;
        public NeutralMode kAzimuthInitNeutralMode = NeutralMode.Brake; // neutral mode could change
        public double kAzimuthTicksPerRadian = 4096.0 / (2 * Math.PI); // for azimuth
        public double kAzimuthEncoderHomeOffset = 0;

        // azimuth motion
        public double kAzimuthKp = 1.3;
        public double kAzimuthKi = 0.05;
        public double kAzimuthKd = 20;
        public double kAzimuthKf = 0.5421;
        public int kAzimuthIZone = 25;
        public int kAzimuthCruiseVelocity = 1698;
        public int kAzimuthAcceleration = 20379; // 12 * kAzimuthCruiseVelocity
        public int kAzimuthClosedLoopAllowableError = 5;

        // azimuth current/voltage
        public int kAzimuthContinuousCurrentLimit = 30; // amps
        public int kAzimuthPeakCurrentLimit = 60; // amps
        public int kAzimuthPeakCurrentDuration = 200; // ms
        public boolean kAzimuthEnableCurrentLimit = true;
        public double kAzimuthMaxVoltage = 10.0; // volts
        public int kAzimuthVoltageMeasurementFilter = 8; // # of samples in rolling average

        // azimuth measurement
        public int kAzimuthStatusFrame2UpdateRate = 10; // feedback for selected sensor, ms
        public int kAzimuthStatusFrame10UpdateRate = 10; // motion magic, ms
        public VelocityMeasPeriod kAzimuthVelocityMeasurementPeriod = VelocityMeasPeriod.Period_100Ms; // dt for velocity measurements, ms
        public int kAzimuthVelocityMeasurementWindow = 64; // # of samples in rolling average

        // general drive
        public boolean kInvertDrive = true;
        public boolean kInvertDriveSensorPhase = false;
        public NeutralMode kDriveInitNeutralMode = NeutralMode.Brake; // neutral mode could change
        public double kWheelDiameter = 4.0; // Probably should tune for each individual wheel maybe
        public double kDriveTicksPerUnitDistance = (1.0 / 4096.0) * (18.0 / 28.0 * 15.0 / 45.0)
                * (Math.PI * kWheelDiameter);
        public double kDriveDeadband = 0.01;

        // drive current/voltage
        public int kDriveContinuousCurrentLimit = 30; // amps
        public int kDrivePeakCurrentLimit = 50; // amps
        public int kDrivePeakCurrentDuration = 200; // ms
        public boolean kDriveEnableCurrentLimit = true;
        public double kDriveMaxVoltage = 10.0; // volts
        public int kDriveVoltageMeasurementFilter = 8; // # of samples in rolling average

        // drive measurement
        public int kDriveStatusFrame2UpdateRate = 15; // feedback for selected sensor, ms
        public int kDriveStatusFrame10UpdateRate = 200; // motion magic, ms
        public VelocityMeasPeriod kDriveVelocityMeasurementPeriod = VelocityMeasPeriod.Period_100Ms; // dt for velocity measurements, ms
        public int kDriveVelocityMeasurementWindow = 64; // # of samples in rolling average
    }

    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private ControlState mControlState = ControlState.OPEN_LOOP;
    private final SwerveModuleConstants mConstants;

    private TalonSRX mDriveTalon;
    private TalonSRX mAzimuthTalon;

    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;

    public SwerveModule(SwerveModuleConstants constants) {
        mConstants = constants;

        mDriveTalon = TalonSRXFactory.createDefaultTalon(mConstants.kDriveTalonId);
        mAzimuthTalon = TalonSRXFactory.createDefaultTalon(mConstants.kAzimuthTalonId);

        // config sensors
        TalonSRXUtil.checkError(
                mDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0,
                        Constants.kLongCANTimeoutMs),
                "Error in " + mConstants.kName + "Module: Unable to config drive encoder");
        TalonSRXUtil.checkError(
                mAzimuthTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0,
                        Constants.kLongCANTimeoutMs),
                "Error in " + mConstants.kName + "Module: Unable to config azimuth encoder");

        // config azimuth motion
        TalonSRXUtil.checkError(mAzimuthTalon.config_kP(0, mConstants.kAzimuthKp, Constants.kLongCANTimeoutMs),
                "Error in " + mConstants.kName + "Module: Unable to config azimuth kp");
        TalonSRXUtil.checkError(mAzimuthTalon.config_kI(0, mConstants.kAzimuthKi, Constants.kLongCANTimeoutMs),
                "Error in " + mConstants.kName + "Module: Unable to config azimuth ki");
        TalonSRXUtil.checkError(
                mAzimuthTalon.config_IntegralZone(0, mConstants.kAzimuthIZone, Constants.kLongCANTimeoutMs),
                "Error in " + mConstants.kName + "Module: Unable to config azimuth i zone");
        TalonSRXUtil.checkError(mAzimuthTalon.config_kD(0, mConstants.kAzimuthKd, Constants.kLongCANTimeoutMs),
                "Error in " + mConstants.kName + "Module: Unable to config azimuth kd");
        TalonSRXUtil.checkError(mAzimuthTalon.config_kF(0, mConstants.kAzimuthKf, Constants.kLongCANTimeoutMs),
                "Error in " + mConstants.kName + "Module: Unable to config azimuth kf");
        TalonSRXUtil.checkError(
                mAzimuthTalon.configMotionCruiseVelocity(mConstants.kAzimuthCruiseVelocity,
                        Constants.kLongCANTimeoutMs),
                "Error in " + mConstants.kName + "Module: Unable to config azimuth cruise vel");
        TalonSRXUtil.checkError(
                mAzimuthTalon.configMotionAcceleration(mConstants.kAzimuthAcceleration, Constants.kLongCANTimeoutMs),
                "Error in " + mConstants.kName + "Module: Unable to config azimuth max acc");
        TalonSRXUtil.checkError(
                mAzimuthTalon.configAllowableClosedloopError(0, mConstants.kAzimuthClosedLoopAllowableError,
                        Constants.kLongCANTimeoutMs),
                "Error in " + mConstants.kName + "Module: Unable to config azimuth allowable closed loop error");
        mAzimuthTalon.selectProfileSlot(0, 0);

        // config azimuth current/voltage settings
        TalonSRXUtil.checkError(
                mAzimuthTalon.configContinuousCurrentLimit(mConstants.kAzimuthContinuousCurrentLimit,
                        Constants.kLongCANTimeoutMs),
                "Error in " + mConstants.kName + "Module: Unable to config azimuth continuous current limit");
        TalonSRXUtil.checkError(
                mAzimuthTalon.configPeakCurrentLimit(mConstants.kAzimuthPeakCurrentLimit, Constants.kLongCANTimeoutMs),
                "Error in " + mConstants.kName + "Module: Unable to config azimuth peak current limit");
        TalonSRXUtil.checkError(
                mAzimuthTalon.configPeakCurrentDuration(mConstants.kAzimuthPeakCurrentDuration,
                        Constants.kLongCANTimeoutMs),
                "Error in " + mConstants.kName + "Module: Unable to config azimuth peak current duration");
        mAzimuthTalon.enableCurrentLimit(mConstants.kAzimuthEnableCurrentLimit);
        TalonSRXUtil.checkError(
                mAzimuthTalon.configVoltageMeasurementFilter(mConstants.kAzimuthVoltageMeasurementFilter,
                        Constants.kLongCANTimeoutMs),
                "Error in " + mConstants.kName + "Module: Unable to config azimuth voltage measurement filter");
        TalonSRXUtil.checkError(
                mAzimuthTalon.configVoltageCompSaturation(mConstants.kAzimuthMaxVoltage, Constants.kLongCANTimeoutMs),
                "Error in " + mConstants.kName + "Module: Unable to config azimuth voltage comp saturation");
        mAzimuthTalon.enableVoltageCompensation(true);

        // config azimuth measurement settings
        TalonSRXUtil.checkError(
                mAzimuthTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,
                        mConstants.kAzimuthStatusFrame2UpdateRate, Constants.kLongCANTimeoutMs),
                "Error in " + mConstants.kName + "Module: Unable to config azimuth status frame 2 period");
        TalonSRXUtil.checkError(
                mAzimuthTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic,
                        mConstants.kAzimuthStatusFrame10UpdateRate, Constants.kLongCANTimeoutMs),
                "Error in " + mConstants.kName + "Module: Unable to config azimuth status frame 10 period");
        TalonSRXUtil.checkError(
                mAzimuthTalon.configVelocityMeasurementPeriod(mConstants.kAzimuthVelocityMeasurementPeriod,
                        Constants.kLongCANTimeoutMs),
                "Error in " + mConstants.kName + "Module: Unable to config azimuth velocity measurement period");
        TalonSRXUtil.checkError(
                mAzimuthTalon.configVelocityMeasurementWindow(mConstants.kAzimuthVelocityMeasurementWindow,
                        Constants.kLongCANTimeoutMs),
                "Error in " + mConstants.kName + "Module: Unable to config azimuth velocity measurement window");

        // config drive current/voltage settings
        TalonSRXUtil.checkError(
                mDriveTalon.configContinuousCurrentLimit(mConstants.kDriveContinuousCurrentLimit,
                        Constants.kLongCANTimeoutMs),
                "Error in " + mConstants.kName + "Module: Unable to config drive continuous current limit");
        TalonSRXUtil.checkError(
                mDriveTalon.configPeakCurrentLimit(mConstants.kDrivePeakCurrentLimit, Constants.kLongCANTimeoutMs),
                "Error in " + mConstants.kName + "Module: Unable to config drive peak current limit");
        TalonSRXUtil.checkError(
                mDriveTalon.configPeakCurrentDuration(mConstants.kDrivePeakCurrentDuration,
                        Constants.kLongCANTimeoutMs),
                "Error in " + mConstants.kName + "Module: Unable to config drive peak current duration");
        mDriveTalon.enableCurrentLimit(mConstants.kDriveEnableCurrentLimit);
        TalonSRXUtil.checkError(
                mDriveTalon.configVoltageMeasurementFilter(mConstants.kDriveVoltageMeasurementFilter,
                        Constants.kLongCANTimeoutMs),
                "Error in " + mConstants.kName + "Module: Unable to config drive voltage measurement filter");
        TalonSRXUtil.checkError(
                mDriveTalon.configVoltageCompSaturation(mConstants.kDriveMaxVoltage, Constants.kLongCANTimeoutMs),
                "Error in " + mConstants.kName + "Module: Unable to config drive voltage comp saturation");
        mDriveTalon.enableVoltageCompensation(true);

        // config drive measurement settings
        TalonSRXUtil.checkError(
                mDriveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,
                        mConstants.kDriveStatusFrame2UpdateRate, Constants.kLongCANTimeoutMs),
                "Error in " + mConstants.kName + "Module: Unable to config drive status frame 2 period");
        TalonSRXUtil.checkError(
                mDriveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic,
                        mConstants.kDriveStatusFrame10UpdateRate, Constants.kLongCANTimeoutMs),
                "Error in " + mConstants.kName + "Module: Unable to config drive status frame 10 period");
        TalonSRXUtil.checkError(
                mDriveTalon.configVelocityMeasurementPeriod(mConstants.kDriveVelocityMeasurementPeriod,
                        Constants.kLongCANTimeoutMs),
                "Error in " + mConstants.kName + "Module: Unable to config drive velocity measurement period");
        TalonSRXUtil.checkError(
                mDriveTalon.configVelocityMeasurementWindow(mConstants.kDriveVelocityMeasurementWindow,
                        Constants.kLongCANTimeoutMs),
                "Error in " + mConstants.kName + "Module: Unable to config drive velocity measurement window");

        // config general drive settings
        mDriveTalon.setInverted(mConstants.kInvertDrive);
        mDriveTalon.setSensorPhase(mConstants.kInvertDriveSensorPhase);
        mDriveTalon.setNeutralMode(mConstants.kDriveInitNeutralMode);
        TalonSRXUtil.checkError(mDriveTalon.configForwardSoftLimitEnable(false, Constants.kLongCANTimeoutMs),
                "Error in " + mConstants.kName + "Module: Unable to disable drive forward soft limit");
        TalonSRXUtil.checkError(mDriveTalon.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs),
                "Error in " + mConstants.kName + "Module: Unable to disable drive reverse soft limit");

        // config general azimuth settings
        mAzimuthTalon.setInverted(mConstants.kInvertAzimuth);
        mAzimuthTalon.setSensorPhase(mConstants.kInvertAzimuthSensorPhase);
        mAzimuthTalon.setNeutralMode(mConstants.kAzimuthInitNeutralMode);
        TalonSRXUtil.checkError(mAzimuthTalon.configForwardSoftLimitEnable(false, Constants.kLongCANTimeoutMs),
                "Error in " + mConstants.kName + "Module: Unable to disable azimuth forward soft limit");
        TalonSRXUtil.checkError(mAzimuthTalon.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs),
                "Error in " + mConstants.kName + "Module: Unable to disable azimuth reverse soft limit");

        zeroSensors();
    }

    public synchronized void setOpenLoop(double speed, Rotation2d azimuth) {
        if (mControlState != ControlState.OPEN_LOOP) {
            mControlState = ControlState.OPEN_LOOP;
        }

        Rotation2d current = getAngle();

        double raw_error = current.distance(azimuth);
        if (Math.abs(raw_error) > Math.PI) {
            raw_error -= (Math.PI * 2 * Math.signum(raw_error));
        }

        // error is -180 to 180
        // is wheel reversible logic
        if (Math.abs(raw_error) > Math.PI / 2) {
            speed *= -1;
            raw_error -= Math.PI * Math.signum(raw_error);
        }

        double final_setpoint = getRawAngle() + raw_error;
        // double adjusted_speed = speed * Math.abs(Math.cos(raw_error));

        mPeriodicIO.drive_demand = speed;
        mPeriodicIO.azimuth_demand = radiansToEncoderUnits(final_setpoint);
    }

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.drive_encoder_ticks = mDriveTalon.getSelectedSensorPosition(0);
        mPeriodicIO.distance = (int) encoderUnitsToDistance(mPeriodicIO.drive_encoder_ticks);
        mPeriodicIO.velocity_ticks_per_100ms = mDriveTalon.getSelectedSensorVelocity(0);
        mPeriodicIO.azimuth_encoder_ticks = mAzimuthTalon.getSelectedSensorPosition(0)
                - mConstants.kAzimuthEncoderHomeOffset;

        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
    }

    @Override
    public void writePeriodicOutputs() {
        if (mControlState == ControlState.OPEN_LOOP) {
            if (Util.epsilonEquals(mPeriodicIO.drive_demand, 0.0, mConstants.kDriveDeadband)) { // don't move if
                // throttle is 0
                stop();
            } else {
                mAzimuthTalon.set(ControlMode.MotionMagic,
                        mPeriodicIO.azimuth_demand + mConstants.kAzimuthEncoderHomeOffset);
                mDriveTalon.set(ControlMode.PercentOutput, mPeriodicIO.drive_demand);
            }
        }
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                synchronized (SwerveModule.this) {
                    stop();
                    startLogging();
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (SwerveModule.this) {
                    switch (mControlState) {
                        case OPEN_LOOP:
                            break;
                        default:
                            System.out.println("Unexpected control state: " + mControlState);
                            break;
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {
                stop();
                stopLogging();
            }
        });
    }

    @Override
    public void zeroSensors() {
        mDriveTalon.setSelectedSensorPosition(0, 0, Constants.kCANTimeoutMs);
        /* Azimuth Talon should be in absolute mode */
    }

    @Override
    public void stop() {
        mDriveTalon.set(ControlMode.PercentOutput, 0.0);
        mAzimuthTalon.set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber(mConstants.kName + " Module: Module Angle", getAngle().getDegrees());
        SmartDashboard.putNumber(mConstants.kName + " Module: Linear Velocity", getLinearVelocity());
        SmartDashboard.putNumber(mConstants.kName + " Module: Distance Driven", mPeriodicIO.distance);

        SmartDashboard.putNumber(mConstants.kName + " Module: Drive Demand", mPeriodicIO.drive_demand);
        SmartDashboard.putNumber(mConstants.kName + " Module: Azimuth Demand",
                Math.toDegrees(Util.bound0To2PIRadians(encoderUnitsToRadians(mPeriodicIO.azimuth_demand))));
        SmartDashboard.putNumber(mConstants.kName + " Module: Azimuth Error",
                Math.toDegrees(encoderUnitsToRadians(mPeriodicIO.azimuth_demand - getAngleEncoderUnits())));

        SmartDashboard.putNumber(mConstants.kName + " Module: Actual Drive Percent Output", getDrivePercentOutput());
        SmartDashboard.putBoolean(mConstants.kName + " Module: Drive Demand Equals Actual",
                Util.epsilonEquals(mPeriodicIO.drive_demand, getDrivePercentOutput()));

        SmartDashboard.putBoolean(mConstants.kName + " Module: Azimuth At Target", isAzimuthAtTarget());
        SmartDashboard.putNumber(mConstants.kName + " Module: Current", mAzimuthTalon.getOutputCurrent());
        SmartDashboard.putNumber(mConstants.kName + " Module: Voltage", mAzimuthTalon.getMotorOutputVoltage());

        SmartDashboard.putNumber(mConstants.kName + " Module: Azimuth Absolute Encoder Reading",
                mAzimuthTalon.getSelectedSensorPosition());

        if (mCSVWriter != null) {
            mCSVWriter.write();
        }
    }

    /**
     * @param azimuth ticks
     */
    public synchronized double encoderUnitsToRadians(double ticks) {
        return ticks / mConstants.kAzimuthTicksPerRadian;
    }

    /**
     * @return azimuth ticks
     */
    public synchronized double radiansToEncoderUnits(double radians) {
        return radians * mConstants.kAzimuthTicksPerRadian;
    }

    /**
     * @param drive ticks
     */
    public synchronized double encoderUnitsToDistance(double ticks) {
        return ticks * mConstants.kDriveTicksPerUnitDistance;
    }

    /**
     * @return drive ticks
     */
    public synchronized double distanceToEncoderUnits(double distance) {
        return distance / mConstants.kDriveTicksPerUnitDistance;
    }

    public synchronized double getAngleEncoderUnits() {
        return mPeriodicIO.azimuth_encoder_ticks;
    }

    public synchronized Rotation2d getAngle() {
        return Rotation2d.fromRadians((encoderUnitsToRadians(getAngleEncoderUnits())));
    }

    public synchronized double getRawAngle() {
        return encoderUnitsToRadians(getAngleEncoderUnits());
    }

    public synchronized double getUnwrappedAngleDegrees() {
        return Math.toDegrees(encoderUnitsToRadians(getAngleEncoderUnits()));
    }

    public synchronized double getRawLinearVelocity() {
        return mPeriodicIO.velocity_ticks_per_100ms * 10;
    }

    public synchronized double getLinearVelocity() {
        return encoderUnitsToDistance(getRawLinearVelocity());
    }

    public synchronized void setDriveBrakeMode(boolean brake_mode) {
        mDriveTalon.setNeutralMode(brake_mode ? NeutralMode.Brake : NeutralMode.Coast);
    }

    public synchronized void setAzimuthBrakeMode(boolean brake_mode) {
        mAzimuthTalon.setNeutralMode(brake_mode ? NeutralMode.Brake : NeutralMode.Coast);
    }

    public synchronized double getDrivePercentOutput() {
        return mDriveTalon.getMotorOutputPercent();
    }

    public synchronized boolean isAzimuthAtTarget() {
        return Util.epsilonEquals(mPeriodicIO.azimuth_demand, getAngleEncoderUnits(),
                mConstants.kAzimuthClosedLoopAllowableError);
    }

    public synchronized void startLogging() {
        if (mCSVWriter == null) {
            mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/" + mConstants.kName + "-MODULE-LOGS.csv",
                    PeriodicIO.class);
        }
    }

    public synchronized void stopLogging() {
        if (mCSVWriter != null) {
            mCSVWriter.flush();
            mCSVWriter = null;
        }
    }
}