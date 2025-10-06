package org.team2059.Wonko.subsystems.drive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Central SwerveModule class.
 * Behaves like a subsystem, but has abstracted IO layer.
 */
public class SwerveModule extends SubsystemBase {
  public final SwerveModuleIO io;

  public final SwerveModuleIOInputsAutoLogged inputs;

  public final int moduleId;

  public SwerveModule(
    int moduleId,
    SwerveModuleIO io
  ) {
    this.moduleId = moduleId;
    this.io = io;

    inputs = new SwerveModuleIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(("Drive/Module" + Integer.toString(moduleId)), inputs);
  }
}