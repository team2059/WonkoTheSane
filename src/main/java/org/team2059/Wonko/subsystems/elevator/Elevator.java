package org.team2059.Wonko.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    private ElevatorIO io;

    private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    public Elevator(ElevatorIO io) {
        this.io = io;
        io.resetEncoder();
    }

    public void setPosition(double position) {
        io.setPosition(position);
    }

    public void setVelocity(double velocity) {
        io.setVelocity(velocity);
    }

    public void stop() {
        io.stop();
    }

    public void setLevel(int level) {
        io.setLevel(level);
    }

    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    public boolean getLimitSwitch(int level) {
        return inputs.limitSwitches[level];
    }

    public Command goToLevelCommand(int level) {
        return this.startEnd(() -> this.setLevel(level), () -> this.stop())
            .until(() -> this.getLimitSwitch(level));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        if (inputs.positionMeters > 2.7) {
            stop();
        }
    }
}
