package org.team2059.Wonko.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleIO {
    @AutoLog
    class SwerveModuleIOInputs {
        public boolean driveConnected = false;
        public boolean rotationConnected = false;

        public double drivePosition = 0.0;
        public double driveVelocity = 0.0;

        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;

        public double rotationAbsolutePositionRadians = 0.0;
        public double rotationPositionRadians = 0.0;
        public double rotationVelocityRadPerSec = 0.0;

        public double rotationAppliedVolts = 0.0;
        public double rotationCurrentAmps = 0.0;

        public double driveMotorTemp = 0.0;
        public double rotationMotorTemp = 0.0;
    }

    default public void updateInputs(SwerveModuleIOInputs inputs) {};

    default public void initRotationOffset() {};

    default public void resetEncoders() {};

    default public void setState(SwerveModuleState state, boolean isClosedLoop) {};

    default public void stop() {};

    default public void setDriveVoltage(double volts) {};
    default public void setRotationVoltage(double volts) {};

    default public SwerveModuleState getState() {
        return new SwerveModuleState();
    }

    default public void setRotationMotorAnglePID(double angleRadians) {};
}
