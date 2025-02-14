package org.team2059.Wonko.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    private ElevatorIO io;

    private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    public Elevator(ElevatorIO io) {
        this.io = io;
        io.resetEncoder();

        setDefaultCommand(Commands.run(() -> io.hold(), this));
    }

    public Command setGoalPosition(double position) {
        return Commands.runOnce(() -> io.setGoal(position), this)
            .andThen(Commands.run(() -> io.updateMotionProfile(), this))
            .withTimeout(2.5); // Makes it timuout after 2.5 seconds
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

    // public boolean getLimitSwitch(int level) {
    //     return inputs.limitSwitches[level];
    // }s

    public Command goToLevelCommand(int level) {
        return this.startEnd(() -> this.setLevel(level), () -> this.stop());
            // .until(() -> this.getLimitSwitch(level));
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
