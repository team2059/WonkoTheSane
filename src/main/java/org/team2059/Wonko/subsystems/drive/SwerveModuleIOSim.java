package org.team2059.Wonko.subsystems.drive;

import org.team2059.Wonko.Constants.DrivetrainConstants;
import org.team2059.Wonko.util.SwerveUtilities;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SwerveModuleIOSim implements SwerveModuleIO {
    private DCMotorSim driveSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getNeoVortex(1), 
            0.025, 
            DrivetrainConstants.driveGearRatio
        ), 
        DCMotor.getNeoVortex(1)
    );

    private DCMotorSim rotationSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getNeoVortex(1), 
            0.004, 
            DrivetrainConstants.rotationGearRatio
        ), 
        DCMotor.getNeoVortex(1)
    );

    private PIDController rotationPidController;

    public SwerveModuleIOSim() {
        rotationPidController = new PIDController(0.25, 0, 0);
        rotationPidController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void resetEncoders() {
        driveSim.setState(0, 0);
        rotationSim.setState(0, 0);
    }

    public double getDriveEncoderPosition() {
        return driveSim.getAngularPositionRotations() * DrivetrainConstants.driveEncoderPositionConversionFactor;
    }

    public double getDriveVelocity() {
        return driveSim.getAngularVelocityRPM() * DrivetrainConstants.driveEncoderVelocityConversionFactor;
    }

    public double getRotationEncoderPosition() {
        return rotationSim.getAngularPositionRotations() * DrivetrainConstants.rotationEncoderPositionConversionFactor;
    }

    public double getRotationEncoderAbsolutePosition() {
        return getRotationEncoderPosition() % (2 * Math.PI);
    }

    public double getRotationVelocity() {
        return rotationSim.getAngularVelocityRPM() * DrivetrainConstants.rotationEncoderVelocityConversionFactor;
    }

    @Override
    public void setState(SwerveModuleState state, boolean isClosedLoop) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        // Optimize angle of state to minmize rotation
        state = SwerveUtilities.optimize(state, Rotation2d.fromRadians(getRotationEncoderAbsolutePosition()));

        // PID-controlled rotation
        setRotationVoltage(rotationPidController.calculate(getRotationEncoderAbsolutePosition(), state.angle.getRadians()));
        setDriveVoltage(DrivetrainConstants.driveFF.calculate(state.speedMetersPerSecond));
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {

        driveSim.update(0.02);
        rotationSim.update(0.02);

        inputs.drivePosition = getDriveEncoderPosition();
        inputs.driveVelocity = getDriveVelocity();

        inputs.driveAppliedVolts = driveSim.getInputVoltage();
        inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

        inputs.rotationAbsolutePositionRadians = getRotationEncoderAbsolutePosition();
        inputs.rotationPositionRadians = getRotationEncoderPosition();
        inputs.rotationVelocityRadPerSec = getRotationVelocity();

        inputs.rotationAppliedVolts = rotationSim.getInputVoltage();
        inputs.rotationCurrentAmps = Math.abs(rotationSim.getCurrentDrawAmps());
    }

    // This is intentionally redundant - no CANcoder in simulation
    @Override
    public void initRotationOffset() {}

    @Override
    public void stop() {
        driveSim.setInputVoltage(0);
        rotationSim.setInputVoltage(0);
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveSim.setInputVoltage(MathUtil.clamp(volts, -12, 12));
    }

    @Override
    public void setRotationVoltage(double volts) {
        rotationSim.setInputVoltage(MathUtil.clamp(volts, -12, 12));
    }
}