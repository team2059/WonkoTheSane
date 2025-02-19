package org.team2059.Wonko.subsystems.elevator;

import org.littletonrobotics.junction.Logger;
import org.team2059.Wonko.routines.ElevatorRoutine;

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

        if (inputs.positionMeters > 2.3) {
            io.stop();
        }
    }
}
