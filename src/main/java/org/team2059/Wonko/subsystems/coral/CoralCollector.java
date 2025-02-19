package org.team2059.Wonko.subsystems.coral;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralCollector extends SubsystemBase {

    public CoralCollectorIO io;
    public CoralCollectorIOInputsAutoLogged inputs;

    public CoralCollector(CoralCollectorIO io) {
        this.io = io;
        inputs = new CoralCollectorIOInputsAutoLogged();
    }

    public Command intakeCommand() {
        return this.startEnd(() -> io.setIntakeSpeed(1000), () -> io.stopIntake());
            // .until(() -> inputs.hasCoral);
    }

    public Command outtakeCommand() {
        return this.startEnd(() -> io.setIntakeSpeed(0), () -> io.stopIntake());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("CoralCollector", inputs);
    }
}