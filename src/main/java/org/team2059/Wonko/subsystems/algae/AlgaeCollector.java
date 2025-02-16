package org.team2059.Wonko.subsystems.algae;

import org.littletonrobotics.junction.Logger;
import org.team2059.Wonko.Constants.AlgaeCollectorConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeCollector extends SubsystemBase {

    public AlgaeCollectorIO io;
    
    public AlgaeCollectorIOInputsAutoLogged inputs = new AlgaeCollectorIOInputsAutoLogged();

    public AlgaeCollector(AlgaeCollectorIO io) {
        this.io = io;
    }

    public Command tiltCommand() {
        return new InstantCommand();
    }

    public Command intakeCommand() {
        return this.startEnd(() -> io.setIntakeSpeed(0.25), () -> io.setIntakeSpeed(AlgaeCollectorConstants.holdSpeed))
            .until(() -> inputs.hasAlgae);
    }

    public Command outtakeCommand() {
        return this.startEnd(() -> io.setIntakeSpeed(-0.25), () -> io.stopIntake());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("AlgaeCollector", inputs);
    }
}
