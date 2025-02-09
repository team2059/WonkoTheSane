// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2059.Wonko.commands;

import org.team2059.Wonko.Constants.AlgaeIntakeConstants;
import org.team2059.Wonko.subsystems.Algae.AlgaeIntake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetAlgaeTiltCmd extends Command {
  private final AlgaeIntake algaeIntake;
  private final PIDController pidController;
  private final double targetPosition;
  
  /** Creates a new SetCoralTiltCmd. */
  public SetAlgaeTiltCmd(AlgaeIntake algaeIntake, double targetPosition) {
    // Creates algaeIntake, target (tilt degrees of the algae), and PID controller for smooth tilting
    this.algaeIntake = algaeIntake;
    this.targetPosition = targetPosition;
    this.pidController = new PIDController(AlgaeIntakeConstants.kPAlgae, AlgaeIntakeConstants.kIAlgae, AlgaeIntakeConstants.kDAlgae);
    // Sets tolerance so it won't continue the command once in the position tolerance so it won't continue oscillating forever 
    pidController.setTolerance(AlgaeIntakeConstants.POSITION_TOLERANCE);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(algaeIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Resets all error from PID from previous requests
    pidController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Gets throughbore position
    double currentPosition = algaeIntake.getAbsolutePosition();
    // Gets amount of percent to put by using PID controller to calculate optimally 
    double output = pidController.calculate(currentPosition, targetPosition);
    
    // Max speed is 50% 
    output = Math.min(Math.max(output, -0.5), 0.5);
    
    // Sets pid output to the tilt motor 
    algaeIntake.getTiltMotor().set(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algaeIntake.getTiltMotor().set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Command ends when setpoint is reached (WITHIN THE POSITION TOLERANCE)
    return pidController.atSetpoint();
  }
}