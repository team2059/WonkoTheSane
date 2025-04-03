package org.team2059.Wonko.subsystems.algae;

import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;
import org.team2059.Wonko.routines.AlgaeCollectorRoutine;
import org.team2059.Wonko.util.Elastic;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeCollector extends SubsystemBase {

    public AlgaeCollectorIO io;
    
    public AlgaeCollectorIOInputsAutoLogged inputs = new AlgaeCollectorIOInputsAutoLogged();

    public AlgaeCollectorRoutine routine;

    private Elastic.Notification AlgaeIntakeCompleteNotif = new Elastic.Notification(
        Elastic.Notification.NotificationLevel.INFO,
        "Intake Complete",
        "Algae Collector"
    );

    public AlgaeCollector(AlgaeCollectorIO io) {
        this.io = io;

        routine = new AlgaeCollectorRoutine(this);
    }

    public Command intakeCommand() {
        return Commands.startEnd(() -> io.setIntakeSpeed(-0.75), () -> io.setIntakeSpeed(0))
            .until(() -> inputs.hasAlgae)
            .andThen(new InstantCommand(() -> {
                Elastic.sendNotification(AlgaeIntakeCompleteNotif);
            }));
    }

    public Command outtakeCommand() {
        return Commands.startEnd(() -> io.setIntakeSpeed(0.75), () -> io.stopIntake());
    }

    public Command setTiltSetpointCmd(Angle targetAngle) {
        return this.run(
            () -> io.setTiltPos(targetAngle.in(Radians))
        );
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("AlgaeCollector", inputs);
    }
}
