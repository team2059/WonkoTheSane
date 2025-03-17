package org.team2059.Wonko.subsystems.coral;

import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;
import org.team2059.Wonko.routines.CoralCollectorRoutine;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralCollector extends SubsystemBase {

    /* Singleton Instance */
    private static CoralCollector instance;

    public CoralCollectorIO io;
    public CoralCollectorIOInputsAutoLogged inputs;
    public CoralCollectorRoutine routine;

    private CoralCollector(CoralCollectorIO io) {
        this.io = io;
        inputs = new CoralCollectorIOInputsAutoLogged();
        routine = new CoralCollectorRoutine(this);
    }

    public static CoralCollector getInstance(CoralCollectorIO io) {
        if (instance == null) {
            instance = new CoralCollector(io);
        }
        return instance;
    }

    public Command intakeCommand() { // Command factory for coral intake.
        return Commands.startEnd(() -> io.setIntakeSpeed(1000), () -> io.setIntakeSpeed(0))
            .until(() -> inputs.hasCoral);
    }

    public Command outtakeCommand() { // Command factory for coral outtake/release.
        return Commands.startEnd(() -> io.setIntakeSpeed(-1000), () -> io.setIntakeSpeed(0));
    }

    public Command setTiltSetpointCmd(Angle targetAngle) {
        return Commands.run(
            () -> io.setTiltPos(targetAngle.in(Radians))
        );
    }

    @Override
    public void periodic() {
        // Update inputs & publish to logger
        io.updateInputs(inputs);
        Logger.processInputs("CoralCollector", inputs);
    }
}
