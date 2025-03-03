package org.team2059.Wonko.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {

    public final SwerveModuleIO io;
    public final int id;

    public final SwerveModuleIOInputsAutoLogged inputs;

    public SwerveModule(
        int id,
        SwerveModuleIO io
    ) {
        this.id = id;
        this.io = io;

        inputs = new SwerveModuleIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(("Drive/Module" + Integer.toString(id)), inputs);
    }
}
