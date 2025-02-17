package org.team2059.Wonko.subsystems.climber;

import org.team2059.Wonko.subsystems.climber.ClimberIO;
import org.team2059.Wonko.subsystems.climber.ClimberIOInputsAutoLogged;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{
    public ClimberIO io;
    
    public ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    public Climber(ClimberIO io) {
        this.io = io;
    }

    public Command climberDownCommand() {
        return this.startEnd(() -> io.setClimbSpeed(0.05), () -> io.stopClimb());                            
    }

    public Command climberUpCommand() {
        return this.startEnd(() -> io.setClimbSpeed(-0.05), () -> io.stopClimb());
    }
}
