// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2059.Wonko.commands;

import org.team2059.Wonko.subsystems.Coral.CoralIntake;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralIntakeCmd extends Command {
  private final CoralIntake coralIntake;
  public CoralIntakeCmd(CoralIntake coralIntake) {
    this.coralIntake = coralIntake;
    addRequirements(coralIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    coralIntake.setIntakeSpeed(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Uses method from ir sensor to check if coral is in the intake 
    if (coralIntake.isCoralPresent()) {
      // If coral exists already then motor shouldn't run
      coralIntake.setIntakeSpeed(0);
    } else {
      // If coral isn't in the intake, start intaking 
      coralIntake.setIntakeSpeed(0.3);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Set coral to 0 when interrupted (outtake cmd is used)
    coralIntake.setIntakeSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
