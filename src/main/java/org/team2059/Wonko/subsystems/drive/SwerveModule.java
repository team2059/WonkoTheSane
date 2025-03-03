package org.team2059.Wonko.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {

    public final SwerveModuleIO io;
    private final int id;

    public final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();

    public SwerveModule(
        int id,
        SwerveModuleIO io
    ) {
        this.id = id;
        this.io = io;
    }

    public void stop() {
        io.stop();
    }

    public double getDrivePositionMeters() {
        return inputs.drivePosition;
    }

    public double getDriveVelocity() {
        return inputs.driveVelocity;
    }

    public double getRotationAbsolutePosition() {
        return inputs.rotationAbsolutePositionRadians;
    }

    public double getRotationPosition() {
        return inputs.rotationPositionRadians;
    }

    public double getRotationVelocity() {
        return inputs.rotationVelocityRadPerSec;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getRotationAbsolutePosition()));
    }

    public void setState(SwerveModuleState state, boolean isClosedLoop) {
        io.setState(state, false);
    }

    public void initRotationOffset() {
        io.initRotationOffset();
    }

    public void resetEncoders() {
        io.resetEncoders();
    }

    public void setDriveVoltage(double voltage) {
        io.setDriveVoltage(voltage);
    }
    public void setRotationVoltage(double voltage) {
        io.setRotationVoltage(voltage);
    }

    public double getDriveAppliedVoltage() {
        return inputs.driveAppliedVolts;
    }

    public double getRotationAppliedVoltage() {
        return inputs.rotationAppliedVolts;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(("Drive/Module" + Integer.toString(id)), inputs);
    }
}
