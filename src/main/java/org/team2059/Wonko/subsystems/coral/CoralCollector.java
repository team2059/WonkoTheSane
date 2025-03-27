package org.team2059.Wonko.subsystems.coral;

import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;
import org.team2059.Wonko.routines.CoralCollectorRoutine;
import org.team2059.Wonko.util.Elastic;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralCollector extends SubsystemBase {

    public CoralCollectorIO io;
    public CoralCollectorIOInputsAutoLogged inputs;
    public CoralCollectorRoutine routine;

    private Elastic.Notification CoralIntakeCompleteNotif = new Elastic.Notification(
        Elastic.Notification.NotificationLevel.INFO,
        "Intake Complete",
        "Coral Collector"
    );

    public CoralCollector(CoralCollectorIO io) {
        this.io = io;
        inputs = new CoralCollectorIOInputsAutoLogged();
        routine = new CoralCollectorRoutine(this);
    }

    public Command intakeCommand() { // Command factory for coral intake.
        return Commands.startEnd(() -> io.setIntakeSpeed(1000), () -> io.setIntakeSpeed(0))
            .until(() -> inputs.hasCoral)
            .andThen(new InstantCommand(() -> {
                Elastic.sendNotification(CoralIntakeCompleteNotif);
            }));
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
