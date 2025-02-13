// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2059.Wonko.commands;

import org.team2059.Wonko.Constants.CoralIntakeConstants;
import org.team2059.Wonko.subsystems.Coral.CoralIntake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetCoralTiltCmd extends Command {
  private final CoralIntake coralIntake;
  private final PIDController pidController;
  private final double targetPosition;

  private static final double kP = 0.1;
  private static final double kI = 0.0;
  private static final double kD = 0.0;

  
  /** Creates a new SetCoralTiltCmd. */
  public SetCoralTiltCmd(CoralIntake coralIntake, double targetPosition) {
    // Adding coral intake and pid controllers 
    this.coralIntake = coralIntake;
    this.targetPosition = targetPosition;
    this.pidController = new PIDController(kP, kI, kD);
    pidController.setTolerance(CoralIntakeConstants.POSITION_TOLERANCE);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(coralIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Getting throughbore position
    double currentPosition = coralIntake.getAbsolutePosition();
    // Getting output by seeing distance between current throughbore and target throughbore position, using pid to calculate 
    double output = pidController.calculate(currentPosition, targetPosition);
    
    // max speed is 50%
    output = Math.min(Math.max(output, -0.5), 0.5);
    
    // Setting tilt motor to output 
    coralIntake.setTiltSpeed(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Sets tilt motor to 0 when command is done 
    coralIntake.setTiltSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Ends command when at setpoint (WITHIN POSITION TOLERANCE)
    return pidController.atSetpoint();
  }
}







