package org.team2059.Wonko.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;
import org.team2059.Wonko.Constants.ElevatorConstants;
import org.team2059.Wonko.routines.ElevatorRoutine;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    public ElevatorIO io;

    public ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    public ElevatorRoutine routine;

    public Elevator(ElevatorIO io) {
        this.io = io;
        io.resetEncoder();
        this.routine = new ElevatorRoutine(this);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }

    public Command goUpCommand() { // command fact. for simple on/off in forward direction
        return this.startEnd(() -> this.io.setSpeed(0.3), () -> this.io.setSpeed(0))
            .until(() -> this.inputs.positionMeters >= ElevatorConstants.maxHeight.in(Meters));
    }

    public Command goDownCommand() { // command fact. for simple on/off in reverse direction
        return this.startEnd(() -> this.io.setSpeed(-0.05), () -> this.io.setSpeed(0))
            .until(() -> this.inputs.positionMeters <= ElevatorConstants.minHeight.in(Meters));
    }

    public Command set5Volts() {
        return this.startEnd(() -> this.io.setVoltage(4), () -> this.io.stop());
    }
}
