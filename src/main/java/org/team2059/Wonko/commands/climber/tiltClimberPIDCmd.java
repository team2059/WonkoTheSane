// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2059.Wonko.commands.climber;

import org.team2059.Wonko.Constants.ClimberConstants;
import org.team2059.Wonko.subsystems.climber.Climber;
import org.team2059.Wonko.util.LoggedTunableNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class tiltClimberPIDCmd extends Command {

  Climber climber;
  double setpoint;  
  PIDController tiltController = new PIDController(ClimberConstants.kP, ClimberConstants.kI, ClimberConstants.kD); 
  

  private LoggedTunableNumber kP = new LoggedTunableNumber("TiltCoralCollectorCmd/kP", 0.0);
  private LoggedTunableNumber kI = new LoggedTunableNumber("TiltCoralCollectorCmd/kI", 0.0);
  private LoggedTunableNumber kD = new LoggedTunableNumber("TiltCoralCollectorCmd/kD", 0.0);
 
  public tiltClimberPIDCmd(Climber climber, double setpoint) {
    this.climber = climber; 
    this.setpoint = setpoint;
    
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tiltController.reset();
    tiltController.setTolerance(1);
    tiltController.setSetpoint(setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (
      kP.hasChanged(hashCode())
      || kI.hasChanged(hashCode())
      || kD.hasChanged(hashCode())
    ) {
      tiltController.setPID(kP.get(), kI.get(), kD.get());
    }

    double output = 
      MathUtil.clamp
        (tiltController.calculate(climber.inputs.tiltThroughborePosition, setpoint), -1.0, 1.0);
        
    climber.io.setClimbSpeed(output);

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.io.setClimbSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return tiltController.atSetpoint();
  }
}
