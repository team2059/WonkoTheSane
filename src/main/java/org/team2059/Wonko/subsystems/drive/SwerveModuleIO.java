package org.team2059.Wonko.subsystems.drive;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

/**
 * Base SwerveModuleIO interface layer.
 *
 * Subclasses represent different implementations
 * in real life and in simulation.
 * Ex: MK4i, MK4n, MK5n, etc.
 */
public interface SwerveModuleIO  {
  @AutoLog
  class SwerveModuleIOInputs {
    public boolean driveConnected = false;
    public double drivePosition = 0.0;
    public double driveVelocity = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveCurrentAmps = 0.0;
    public double driveMotorTemp = 0.0;

    public boolean azimuthConnected = false;
    public double azimuthAbsolutePosition = 0.0;
    public double azimuthPosition = 0.0;
    public double azimuthVelocity = 0.0;
    public double azimuthAppliedVolts = 0.0;
    public double azimuthCurrentAmps = 0.0;
    public double azimuthMotorTemp = 0.0;
  }

  default public void updateInputs(SwerveModuleIOInputs inputs) {};

  default public void initRotationOffset() {};

  default public void resetEncoders() {};

  default public void setState(SwerveModuleState state, boolean isClosedLoop) {};

  default public void stop() {};

  default public void setDriveVoltage(double volts) {};

  default public void setAzimuthVoltage(double volts) {};

  default public SwerveModuleState getState() {return new SwerveModuleState();}

  default public void setAzimuthAngle(double angle) {};
}