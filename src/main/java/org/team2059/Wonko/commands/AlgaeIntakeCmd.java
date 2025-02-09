// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2059.Wonko.commands;

import org.team2059.Wonko.Constants.AlgaeIntakeConstants;
import org.team2059.Wonko.Constants.DrivetrainConstants;
import org.team2059.Wonko.subsystems.Algae.AlgaeIntake;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeIntakeCmd extends Command {
  private final AlgaeIntake algaeIntake;
  
  /* 
   * Debouncer requires a condition to occur for a certain amount of time in order for the boolean to change
   * kRising means the condition should be going from false to true
   */
  private final Debouncer debounce = new Debouncer(0.33, Debouncer.DebounceType.kRising); 

  // Algae cmd instructor 
  public AlgaeIntakeCmd(AlgaeIntake algaeIntake) {
    this.algaeIntake = algaeIntake;
    addRequirements(algaeIntake);
  }

  @Override
  public void initialize() {
    // Start debouncer at false
    debounce.calculate(false);
    // Uses method to start intaking
    algaeIntake.setEndEffectorSpeed(AlgaeIntakeConstants.INTAKE_ALGAE_SPEED);
  }

  @Override
  public void execute() {
    /*
     * Gets current from a motor using getOutputCurrent() and sees if the current is more than a set constant. 
     * If it is more for more than .33 seconds, then it switches the boolean from false to true, which then signals the motors to slowly backdrive to hold the algae
     */
    if (debounce.calculate(algaeIntake.getMotor1().getOutputCurrent() > AlgaeIntakeConstants.INTAKE_STALL_DETECTION)) {
      algaeIntake.setEndEffectorSpeed(AlgaeIntakeConstants.HOLD_ALGAE_SPEED);
      algaeIntake.hasAlgae = true; 
    }
  }

  @Override
  public void end(boolean interrupted) {
    // When code ends make the motors set at 0 and the algae is no longer in the intake
    algaeIntake.setEndEffectorSpeed(0);
    algaeIntake.hasAlgae = false; 
  }

  @Override
  public boolean isFinished() {
    return false; 
  }
}