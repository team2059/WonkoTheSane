package org.team2059.Wonko.subsystems.drive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyro extends SubsystemBase {
    public GyroIOInputsAutoLogged inputs;
    public GyroIO io;

    public Gyro(GyroIO io) {
        this.io = io;

        inputs = new GyroIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }
}
