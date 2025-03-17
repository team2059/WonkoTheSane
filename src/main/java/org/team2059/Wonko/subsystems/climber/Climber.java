package org.team2059.Wonko.subsystems.climber;

import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;
import org.team2059.Wonko.Constants.ClimberConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{

    /* Singleton Instance */
    private static Climber instance;
    
    public ClimberIO io;
    
    public ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    private Climber(ClimberIO io) {
        this.io = io;
    }

    public static Climber getInstance(ClimberIO io) {
        if (instance == null) {
            instance = new Climber(io);
        }
        return instance;
    } 

    public Command climberDownCommand() {
        return this.startEnd(() -> io.setClimbSpeed(0.3), () -> io.stopClimb())
            .until(() -> inputs.tiltThroughborePosition <= ClimberConstants.lowerLimit.in(Radians));                         
    }

    public Command climberUpCommand() {
        return this.startEnd(() -> io.setClimbSpeed(-0.3), () -> io.stopClimb())
            .until(() -> inputs.tiltThroughborePosition >= ClimberConstants.upperLimit.in(Radians));
    }

    @Override 
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
    }
}
