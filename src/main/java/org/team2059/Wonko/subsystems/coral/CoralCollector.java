package org.team2059.Wonko.subsystems.coral;

import org.littletonrobotics.junction.Logger;
import org.team2059.Wonko.routines.CoralCollectorRoutine;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralCollector extends SubsystemBase {

    public CoralCollectorIO io;
    public CoralCollectorIOInputsAutoLogged inputs;
    public CoralCollectorRoutine routine;

    public CoralCollector(CoralCollectorIO io) {
        this.io = io;
        inputs = new CoralCollectorIOInputsAutoLogged();
        routine = new CoralCollectorRoutine(this);
    }

    public Command intakeCommand() { // Command factory for coral intake.
        return this.startEnd(() -> io.setIntakeSpeed(500), () -> io.setIntakeSpeed(0))
            .until(() -> inputs.hasCoral);
    }

    public Command outtakeCommand() { // Command factory for coral outtake/release.
        return this.startEnd(() -> io.setIntakeSpeed(-500), () -> io.setIntakeSpeed(0));
    }

    @Override
    public void periodic() {
        // Update inputs & publish to logger
        io.updateInputs(inputs);
        Logger.processInputs("CoralCollector", inputs);
    }
}