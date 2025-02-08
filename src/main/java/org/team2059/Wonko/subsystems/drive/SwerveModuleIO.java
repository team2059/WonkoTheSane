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

    public void updateInputs(SwerveModuleIOInputs inputs);

    public void initRotationOffset();

    public void resetEncoders();

    public void setState(SwerveModuleState state, boolean isClosedLoop);

    public void stop();

    public void setDriveVoltage(double volts);
    public void setRotationVoltage(double volts);
}
