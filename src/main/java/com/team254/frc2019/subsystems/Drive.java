package com.team254.frc2019.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.team254.frc2019.Constants;
import com.team254.frc2019.loops.ILooper;
import com.team254.frc2019.loops.Loop;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.util.SwerveDriveHelper;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive extends Subsystem {
    private static Drive mInstance;

    public static Drive getInstance() {
        if (mInstance == null) {
            mInstance = new Drive();
        }

        return mInstance;
    }

    public static class PeriodicIO {
        // INPUTS
        public Rotation2d gyro_heading = Rotation2d.identity();
        public double timestamp = Timer.getFPGATimestamp(); // used for vel and acc

        // Driver controls
        public double forward;
        public double strafe;
        public double rotation;
        public boolean low_power;
        public boolean field_relative;
        public boolean use_heading_controller;

        // OUTPUTS
        public double[] wheel_speeds = new double[] {0, 0, 0, 0};
        public Rotation2d[] wheel_azimuths = new Rotation2d[]{Rotation2d.identity(), Rotation2d.identity(), Rotation2d.identity(), Rotation2d.identity()};
    }

    public enum ControlState {
        OPEN_LOOP
    }

    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private ControlState mControlState = ControlState.OPEN_LOOP;

    private PigeonIMU mPigeonIMU;
    private SwerveModule[] mModules = new SwerveModule[4];

    private Rotation2d mGyroOffset = Rotation2d.identity();

    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;


    private Drive() {
        mPigeonIMU = new PigeonIMU(Constants.kPigeonId);

        mModules[0] = new SwerveModule(Constants.kFrontLeftModuleConstants);
        mModules[1] = new SwerveModule(Constants.kFrontRightModuleConstants);
        mModules[2] = new SwerveModule(Constants.kBackRightModuleConstants);
        mModules[3] = new SwerveModule(Constants.kBackLeftModuleConstants);
    }

    public synchronized void setOpenLoop(DriveSignal signal) {
        if (mControlState != ControlState.OPEN_LOOP) {
            mControlState = ControlState.OPEN_LOOP;
        }

        mPeriodicIO.wheel_speeds = signal.getWheelSpeeds();
        mPeriodicIO.wheel_azimuths = signal.getWheelAzimuths();
    }

    public void setTeleopInputs(double forward, double strafe, double rotation, boolean low_power, boolean field_relative, boolean use_heading_controller) {
        if (mControlState != ControlState.OPEN_LOOP) {
            mControlState = ControlState.OPEN_LOOP;
        }
        mPeriodicIO.forward = forward;
        mPeriodicIO.strafe = strafe;
        mPeriodicIO.rotation = rotation;
        mPeriodicIO.low_power = low_power;
        mPeriodicIO.field_relative = field_relative;
        mPeriodicIO.use_heading_controller = use_heading_controller;
    }

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.gyro_heading = Rotation2d.fromDegrees(mPigeonIMU.getFusedHeading()).rotateBy(mGyroOffset);

        mPeriodicIO.timestamp = Timer.getFPGATimestamp();

        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
    }

    @Override
    public void writePeriodicOutputs() {
        for (int i = 0; i < mModules.length; i++) {
            if (mModules != null && mModules[i] != null) {
                mModules[i].setOpenLoop(mPeriodicIO.wheel_speeds[i], mPeriodicIO.wheel_azimuths[i]);
            }
        }
    }


    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                synchronized (Drive.this) {
                    startLogging();
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Drive.this) {
                    switch (mControlState) {
                        case OPEN_LOOP:
                            setOpenLoop(SwerveDriveHelper.calculateDriveSignal(mPeriodicIO.forward,
                                    mPeriodicIO.strafe, mPeriodicIO.rotation, mPeriodicIO.low_power,
                                    mPeriodicIO.field_relative, mPeriodicIO.use_heading_controller));
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

    public SwerveModule[] getSwerveModules() {
        return mModules;
    }

    @Override
    public void zeroSensors() {
        setHeading(Rotation2d.identity());

        for (int i = 0; i < mModules.length; i++) {
            if (mModules != null && mModules[i] != null) {
                mModules[i].zeroSensors();
            }
        }
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Gyro Heading", getHeading().getDegrees());
        SmartDashboard.putNumber("FW", mPeriodicIO.forward);
        SmartDashboard.putNumber("STR", mPeriodicIO.strafe);
        SmartDashboard.putNumber("Rotation", mPeriodicIO.rotation);
        if (mCSVWriter != null) {
            mCSVWriter.write();
        }
    }

    public Rotation2d getHeading() {
        return mPeriodicIO.gyro_heading;
    }

    public synchronized void setHeading(Rotation2d heading) {
        mGyroOffset = heading.rotateBy(Rotation2d.fromDegrees(mPigeonIMU.getFusedHeading()).inverse());

        mPeriodicIO.gyro_heading = heading;
    }

    public synchronized double[] getModuleVelocities() {
        double[] ret_val = new double[mModules.length];
        for (int i = 0; i < ret_val.length; i++) {
            ret_val[i] = mModules[i].getLinearVelocity();
        }

        return ret_val;
    }

    public synchronized Rotation2d[] getModuleAzimuths() {
        Rotation2d[] ret_val = new Rotation2d[mModules.length];
        for (int i = 0; i < ret_val.length; i++) {
            ret_val[i] = mModules[i].getAngle();
        }

        return ret_val;
    }

    public synchronized void startLogging() {
        if (mCSVWriter == null) {
            mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/DRIVE-LOGS.csv", PeriodicIO.class);
        }
    }

    public synchronized void stopLogging() {
        if (mCSVWriter != null) {
            mCSVWriter.flush();
            mCSVWriter = null;
        }
    }
}