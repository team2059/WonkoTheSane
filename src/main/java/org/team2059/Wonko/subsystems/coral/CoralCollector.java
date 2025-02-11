package org.team2059.Wonko.subsystems.coral;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralCollector extends SubsystemBase {

    private CoralCollectorIO io;
    private CoralCollectorIOInputsAutoLogged inputs;

    public CoralCollector(CoralCollectorIO io) {
        this.io = io;
        inputs = new CoralCollectorIOInputsAutoLogged();
    }

    public Command tiltCommand() {
        return new InstantCommand();
    }

    public Command intakeCommand() {
        return this.startEnd(() -> io.setIntakeSpeed(0.25), () -> io.stopIntake())
            .until(() -> inputs.hasCoral);
    }

    public Command outtakeCommand() {
        return this.startEnd(() -> io.setIntakeSpeed(-0.25), () -> io.stopIntake());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("CoralCollector", inputs);
    }
}