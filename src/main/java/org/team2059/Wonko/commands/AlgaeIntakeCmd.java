// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2059.Wonko.commands;

import org.team2059.Wonko.Constants.DrivetrainConstants;
import org.team2059.Wonko.subsystems.AlgaeIntake;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeIntakeCmd extends Command {
  private final AlgaeIntake algaeIntake;
  private final Debouncer debounce = new Debouncer(0.33, Debouncer.DebounceType.kRising); 

  

  public AlgaeIntakeCmd(AlgaeIntake algaeIntake) {
    this.algaeIntake = algaeIntake;
    addRequirements(algaeIntake);
  }

  @Override
  public void initialize() {
    debounce.calculate(false);
    algaeIntake.setEndEffectorSpeed(DrivetrainConstants.INTAKE_CUBE_SPEED);
  }

  @Override
  public void execute() {
    if (debounce.calculate(algaeIntake.getMotor1().getOutputCurrent() > DrivetrainConstants.INTAKE_STALL_DETECTION)) {
      algaeIntake.setEndEffectorSpeed(DrivetrainConstants.HOLD_CUBE_SPEED);
    }
  }

  @Override
  public void end(boolean interrupted) {
    algaeIntake.setEndEffectorSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false; 
  }
}