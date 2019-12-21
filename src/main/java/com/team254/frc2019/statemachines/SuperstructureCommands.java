package com.team254.frc2019.statemachines;

import com.team254.frc2019.Constants;
import com.team254.frc2019.GamePiece;
import com.team254.frc2019.states.SuperstructureGoal;
import com.team254.frc2019.states.SuperstructureState;
import com.team254.frc2019.subsystems.EndEffector;
import com.team254.frc2019.subsystems.Superstructure;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.Util;

/**
 * Commands for the Superstructure to go to predetermined states or vision based
 * states
 */
public class SuperstructureCommands {
    private static boolean mCargoShipPosition = false;
    private static boolean mMiddlePosition = false;
    private static boolean mHighPosition = false;
    private static boolean mLowPosition = false;

    private SuperstructureCommands() {}

    private static void observeDisk() {
        EndEffector.getInstance().updateObservedGamePiece(GamePiece.DISK);
    }

    private static void observeBall() {
        EndEffector.getInstance().updateObservedGamePiece(GamePiece.BALL);
    }

    public static double goToAutoScore() {
        return goToAutoScore(-1.0);
    }

    public static double goToAutoScore(double offsetOverride) {

        System.out.println("go to auto score called");

        Superstructure ss = Superstructure.getInstance();
        EndEffector end_effector = EndEffector.getInstance();

        SuperstructureGoal lastCommand = ss.getGoal();
        if (lastCommand == null) {
            System.out.println("No comamnd");
            return Double.NaN;
        }

        Translation2d end_effector_pos = lastCommand.state.getPlanarWristJointLocation();

        double offset = offsetOverride;
        double allowedHeightLoss = Double.NaN;
        if (end_effector.getObservedGamePiece() == GamePiece.BALL) {
            if (SuperstructureCommands.isInCargoShipPosition()) {
                offset = 1.0;
            } else if (SuperstructureCommands.isInLowPosition()) {
                // Low ball suffers from limelight tilt set high to avoid colliding with rocket.
                offset = 5.0;
            } else if (SuperstructureCommands.isInMiddlePosition()) {
                offset = 1.0;
            } else if (SuperstructureCommands.isInHighPosition()) {
                offset = 1.0;
                allowedHeightLoss = 2.0;
            }
        } else if (end_effector.getObservedGamePiece() == GamePiece.DISK) {
            if (SuperstructureCommands.isInLowPosition()) {
                offset = 0.0;
            } else if (SuperstructureCommands.isInHighPosition()) {
                offset = -2.0;
                allowedHeightLoss = 4.0;
            }
        }

        double desired_z;
        if (ss.getElevatorControlMode() == Superstructure.ElevatorControlModes.PRISMATIC_WRIST) {
            desired_z = ss.getHeightForWristLevel();
        } else {
            desired_z = end_effector_pos.y();
        }

        double range = ss.getCorrectedRangeToTarget();
        double desired_x =
                range - Constants.kTurretToArmOffset - Math.cos(Math.toRadians(lastCommand.state.wrist)) * Constants.kWristToTremorsEnd;

        if (ss.getElevatorControlMode() != Superstructure.ElevatorControlModes.PRISMATIC_WRIST
                && end_effector_pos.x() >= desired_x - offset) {
            // Already past position, just shoot.
        } else {
            setEndEffectorPos(desired_x - offset, desired_z, allowedHeightLoss);
        }


        // Return the remaining distance.
        return desired_x - offset - ss.getCurrentState().getPlanarWristJointLocation().x();
    }

    public static boolean setEndEffectorPos(double x, double z, double allowedHeightLoss) {
        boolean reachable = true;
        Superstructure ss = Superstructure.getInstance();
        SuperstructureGoal lastCommand = ss.getGoal();
        if (lastCommand == null) {
            return false;
        }
        if (x < 0 || x > Constants.kArmLength) {
            reachable = false;
        }

        x = Util.limit(x, 0, Constants.kArmLength);
        double shoulder_angle_rads = Math.acos(x / Constants.kArmLength);
        if (Double.isNaN(shoulder_angle_rads)) {
            System.out.println("Unreachable jog!");
            return false;
        }

        if (Math.abs(lastCommand.state.shoulder) > 1.0) {
            shoulder_angle_rads *= Math.signum(lastCommand.state.shoulder);
        }

        double elevator_height_diff = Math.sin(shoulder_angle_rads) * Constants.kArmLength;
        double elevator_height = z - elevator_height_diff;

        if (Double.isNaN(elevator_height)) {
            System.out.println("Unreachable jog elevator height!");
            return false;
        }

        if (elevator_height < 0 || elevator_height > Constants.kElevatorConstants.kMaxUnitsLimit) {
            reachable = false;

            elevator_height =
                    Util.limit(elevator_height, 0, Constants.kElevatorConstants.kMaxUnitsLimit);

            if (Double.isNaN(allowedHeightLoss)) {
                // Maintain z, adjust x to make this work.
                shoulder_angle_rads = Math.asin((z - elevator_height) / Constants.kArmLength);
            } else {
                double adjusted_z = z - Math.signum(z - elevator_height_diff) * allowedHeightLoss;
                double adjusted_angle_rads =
                        Math.asin((adjusted_z - elevator_height) / Constants.kArmLength);
                // Take the shoulder angle with a larger magnitude in the case that we don't need to
                // lose all the allowed height loss.
                if (Math.abs(adjusted_angle_rads) > Math.abs(shoulder_angle_rads)) {
                    shoulder_angle_rads = adjusted_angle_rads;
                }
            }

            if (Double.isNaN(shoulder_angle_rads)) {
                System.out.println("Unreachable jog after correction!");
                return false;
            }
        }

        SuperstructureState newCommand = new SuperstructureState(
                elevator_height,
                Math.toDegrees(shoulder_angle_rads),
                lastCommand.state.wrist);

        System.out.println("Setting prismatic wrist.");
        ss.setHeightForWristLevel(z);
        ss.setGoal(new SuperstructureGoal(newCommand),
                Superstructure.ElevatorControlModes.PRISMATIC_WRIST);
        return reachable;
    }

    public static void jogElevator(double delta) {
        Superstructure ss = Superstructure.getInstance();
        ss.jogElevator(delta);
    }

    // Both of these are in in.
    public static void jogEndEffector(double jogX, double jogZ) {
        Superstructure ss = Superstructure.getInstance();
        SuperstructureGoal lastCommand = ss.getGoal();
        if (lastCommand == null) {
            return;
        }

        if (Util.epsilonEquals(jogX, 0.0)) {
            double elevator_height = lastCommand.state.elevator + jogZ;
            elevator_height =
                    Util.limit(elevator_height, 0.0, Constants.kElevatorConstants.kMaxUnitsLimit);
            SuperstructureState newCommand = new SuperstructureState(
                    elevator_height,
                    lastCommand.state.shoulder,
                    lastCommand.state.wrist);
            sendCommandToSuperstructure(newCommand);
            return;
        }

        // Find current end effector positions.
        Translation2d end_effector_pos = lastCommand.state.getPlanarWristJointLocation();
        double jogged_x = end_effector_pos.x() + jogX;
        double jogged_z = end_effector_pos.y() + jogZ;

        if (jogged_x > Constants.kArmLength) {
            // Constrain to max
            jogged_x = Util.limit(jogged_x, 0, Constants.kArmLength);
        }
        jogged_z = Util.limit(jogged_z, 0, Constants.kElevatorConstants.kMaxUnitsLimit);
        setEndEffectorPos(jogged_x, jogged_z, Double.NaN);
    }

    public static SuperstructureState onlyArms(SuperstructureState setpoint) {
        return new SuperstructureState(
                setpoint.elevator,
                setpoint.shoulder,
                setpoint.wrist);
    }

    public static void goToPickupDiskFromWallFar() {
        var intake = IntakeDiskWallFar;
        observeDisk();
        selectPositionByGamepiece(
                onlyArms(intake),
                onlyArms(intake));
    }

    public static void goToPickupDiskFromWall() {
        var intake = IntakeDiskWall;
        observeDisk();
        selectPositionByGamepiece(
                onlyArms(intake),
                onlyArms(intake));
    }

    public static void goToPickupBallFromGround() {
        var intake = IntakeBallGround;
        observeBall();
        sendCommandToSuperstructure(intake);
    }


    public static void goToPickupBallFromWall() {
        var intake = IntakeBallWall;
        observeBall();
        selectPositionByGamepiece(intake, intake);
    }

    public static void goToStowed() {
        sendCommandToSuperstructure(tuckedPosition);
    }


    public static void goToStowedWithBall() {
        selectPositionByGamepiece(
                onlyArms(tuckedPosition),
                onlyArms(tuckedPosition)
        );
    }

    public static void goToScoreLow() {
        selectPositionByGamepiece(
                onlyArms(ScoreBallLow),
                onlyArms(ScoreDiskLow)
        );
        mLowPosition = true;
    }

    public static void goToPreScoreLow() {
        selectPositionByGamepiece(
                onlyArms(PreScoreBallLow),
                onlyArms(PreScoreBallLow)
        );
        mLowPosition = true;
    }

    public static void goToScoreMiddle() {
        selectPositionByGamepiece(
                onlyArms(ScoreBallMiddle),
                onlyArms(ScoreDiskMiddle)
        );
        mMiddlePosition = true;
    }

    public static void goToScoreHigh() {
        selectPositionByGamepiece(
                onlyArms(ScoreBallHigh),
                onlyArms(ScoreDiskHigh)
        );
        mHighPosition = true;
    }

    public static void goToScoreCargo() {
        selectPositionByGamepiece(
                onlyArms(ScoreBallCargoShip),
                onlyArms(ScoreBallCargoShip)
        );
        mCargoShipPosition = true;
    }

    public static void goToPrepareForVacuumClimb() {
        sendCommandToSuperstructure(onlyArms(PrepareForVacuumClimb));
    }

    public static void goToPreAutoThrust() {
        SuperstructureGoal pre_wrist = Superstructure.getInstance().getPreWristLevelGoal();
        if (pre_wrist != null) {
            sendCommandToSuperstructure(onlyArms(pre_wrist.state));
        }
    }

    public static void goToVacuumClimbing() {
        sendCommandToSuperstructure(onlyArms(VacuumClimb));
    }

    public static void goToVacuumHab3ContactPoint() {
        sendCommandToSuperstructure(
                onlyArms(VacuumHab3ContactPoint)
        );
    }

    public static boolean isPreparingToClimb() {
        return false;
    }

    private static void selectPositionByGamepiece(
            SuperstructureState withBall,
            SuperstructureState withDisk) {
        selectPositionByGamepieceWithClimb(withBall, withDisk, withDisk);
    }

    private static void selectPositionByGamepieceWithClimb(
            SuperstructureState withBall,
            SuperstructureState withDisk,
            SuperstructureState whileClimbing) {
        switch (EndEffector.getInstance().getObservedGamePiece()) {
            case BALL:
                sendCommandToSuperstructure(withBall);
                break;
            case DISK:
                sendCommandToSuperstructure(withDisk);
                break;
            case CLIMB:
                sendCommandToSuperstructure(whileClimbing);
                break;
        }

        mCargoShipPosition = false;
        mMiddlePosition = false;
        mHighPosition = false;
        mLowPosition = false;
    }

    public static boolean isInLowPosition() {
        return mLowPosition;
    }

    public static boolean isInHighPosition() {
        return mHighPosition;
    }

    public static boolean isInMiddlePosition() {
        return mMiddlePosition;
    }

    public static boolean isInCargoShipPosition() {
        return mCargoShipPosition;
    }

    private static void sendCommandToSuperstructure(SuperstructureState position) {
        // hmb, hacks ahead
        Superstructure ss = Superstructure.getInstance();
        // ss.setPlannerEnabled();
        ss.setGoal(new SuperstructureGoal(position));
    }

    private static final double kElevatorStowedLowHeight = 12.7;

    public static final SuperstructureState tuckedPosition =
            new SuperstructureState(kElevatorStowedLowHeight, -90, 0);

    public static final SuperstructureState stowedPosition = tuckedPosition;

    static SuperstructureState StowedDisk = stowedPosition;

    public static final double kLevel = 0.0;
    static SuperstructureState IntakeDiskWallFar = new SuperstructureState(1.75, -28.0, kLevel);
    static SuperstructureState IntakeDiskWall = new SuperstructureState(10.65, -70.0, kLevel);
    static SuperstructureState ScoreDiskLow = new SuperstructureState(kElevatorStowedLowHeight, -90, kLevel);
    static SuperstructureState ScoreDiskMiddle = new SuperstructureState(0, 90, kLevel);
    static SuperstructureState ScoreDiskHigh = new SuperstructureState(28.0, 90.0, kLevel);
    static SuperstructureState StowedBall = stowedPosition;

    static SuperstructureState IntakeBallGround = new SuperstructureState(7.5, -53.5, -53.3);

    static SuperstructureState IntakeBallWall = stowedPosition;
    static SuperstructureState ScoreBallLow = new SuperstructureState(22.2, -90, 0);
    static SuperstructureState PreScoreBallLow = new SuperstructureState(14.2, -90, 0);
    static SuperstructureState ScoreBallMiddle = new SuperstructureState(8.2, 90.0, 0);
    static SuperstructureState ScoreBallHigh = new SuperstructureState(Constants.kElevatorConstants.kMaxUnitsLimit, 83, 20.3);
    static SuperstructureState ScoreBallCargoShip = new SuperstructureState(Constants.kElevatorConstants.kMaxUnitsLimit, -90, 0);

    public static SuperstructureGoal PreScoreBallLowGoal = new SuperstructureGoal(PreScoreBallLow);

    // Vacuum climb for champs
    static SuperstructureState PrepareForVacuumClimb = new SuperstructureState(31.0, -90, kLevel);
    public static SuperstructureState VacuumHab3ContactPoint = new SuperstructureState(29.0, -45, kLevel);
    public static SuperstructureState VacuumClimb = new SuperstructureState(4.25, -55, kLevel);
}
