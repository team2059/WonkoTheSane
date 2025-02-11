package org.team2059.Wonko.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

// Collection of methods for angle optimization used during swerve module state calculations.
public class SwerveUtilities {
    public static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % (2.0 * Math.PI);
        if (lowerOffset >= 0) {
          lowerBound = scopeReference - lowerOffset;
          upperBound = scopeReference + ((2.0 * Math.PI) - lowerOffset);
        } else {
          upperBound = scopeReference - lowerOffset;
          lowerBound = scopeReference - ((2.0 * Math.PI) + lowerOffset);
        }
        while (newAngle < lowerBound) {
          newAngle += (2.0 * Math.PI);
        }
        while (newAngle > upperBound) {
          newAngle -= (2.0 * Math.PI);
        }
        if (newAngle - scopeReference > (Math.PI)) {
          newAngle -= (2.0 * Math.PI);
        } else if (newAngle - scopeReference < -(Math.PI)) {
          newAngle += (2.0 * Math.PI);
        }
        return newAngle;
    }

    /**
     * Minimize the change in heading the desired swerve module state would require
     * by potentially
     * reversing the direction the wheel spins. Customized from WPILib's version to
     * include placing in
     * appropriate scope for CTRE and REV onboard control as both controllers as of
     * writing don't have
     * support for continuous input.
     *
     * @param desiredState The desired state.
     * @param currentAngle The current module angle.
     */
    public static SwerveModuleState optimize(
        SwerveModuleState desiredState, Rotation2d currentAngle) {

        double targetAngle = placeInAppropriate0To360Scope(currentAngle.getRadians(), desiredState.angle.getRadians());

        double targetSpeed = desiredState.speedMetersPerSecond;
        double delta = (targetAngle - currentAngle.getRadians());
        if (Math.abs(delta) > (Math.PI / 2)) {
        targetSpeed = -targetSpeed;
        targetAngle = delta > Math.PI / 2 ? (targetAngle -= Math.PI) : (targetAngle += Math.PI);
        }
        return new SwerveModuleState(targetSpeed, new Rotation2d(targetAngle));
    }
}
