package com.team254.lib.util;

import com.team254.frc2019.Kinematics;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

/**
 * Class based on Team 1323's sendInput method to make driving feel better
 */
public class SwerveDriveHelper {
    private final static double kHighAdjustmentPower = 1.75 + 0.4375;
    private final static double kLowAdjustmentPower = 1.50;
    private final static double kMaxSpeed = 1.0;
    private final static double kHighPowerRotationScalar = 0.8;
    private final static double kLowPowerScalar = 0.5;
    private final static double kRotationExponent = 4.0;
    private final static double kPoleThreshold = 0.0;
    private final static double kRobotRelativePoleThreshold = Math.toRadians(5);
    private final static double kDeadband = 0.25;
    private final static double kRotationDeadband = 0.15;

    public static DriveSignal calculateDriveSignal(double forwardInput, double strafeInput, double rotationInput,
                                                   boolean low_power, boolean field_relative, boolean use_heading_controller) {

        Translation2d translationalInput = new Translation2d(forwardInput, strafeInput);
        double inputMagnitude = translationalInput.norm();

        // Snap the translational input to its nearest pole, if it is within a certain
        // threshold of it.

        if (field_relative) {
            if (Math.abs(translationalInput.direction()
                    .distance(translationalInput.direction().nearestPole())) < kPoleThreshold) {
                translationalInput = translationalInput.direction().nearestPole().toTranslation().scale(inputMagnitude);
            }
        } else {
            if (Math.abs(translationalInput.direction()
                    .distance(translationalInput.direction().nearestPole())) < kRobotRelativePoleThreshold) {
                translationalInput = translationalInput.direction().nearestPole().toTranslation().scale(inputMagnitude);
            }
        }


        if (inputMagnitude < kDeadband) {
            translationalInput = new Translation2d();
            inputMagnitude = 0;
        }

        // Scale x and y by applying a power to the magnitude of the vector they create,
        // in order to make the controls less sensitive at the lower end.
        final double power = (low_power) ? kHighAdjustmentPower : kLowAdjustmentPower;
        Rotation2d direction = translationalInput.direction();
        double scaledMagnitude = Math.pow(inputMagnitude, power);
        translationalInput = new Translation2d(direction.cos() * scaledMagnitude, direction.sin() * scaledMagnitude);

        rotationInput = (Math.abs(rotationInput) < kRotationDeadband) ? 0 : rotationInput;
        if (use_heading_controller) { // current constants are tuned to be put to the power of 1.75, and I don't want to retune right now
            rotationInput = Math.pow(Math.abs(rotationInput), 1.75) * Math.signum(rotationInput);
        } else {
            rotationInput = Math.pow(Math.abs(rotationInput), kRotationExponent) * Math.signum(rotationInput);
        }

        translationalInput = translationalInput.scale(kMaxSpeed);
        rotationInput *= kMaxSpeed;

        if (low_power) {
            translationalInput = translationalInput.scale(kLowPowerScalar);
            rotationInput *= kLowPowerScalar;
        } else {
            rotationInput *= kHighPowerRotationScalar;
        }

        return Kinematics.inverseKinematics(translationalInput.x(), translationalInput.y(), rotationInput,
                field_relative);
    }
}