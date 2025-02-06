// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2059.Wonko.commands;

import org.team2059.Wonko.subsystems.Elevator;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorGoToSetpoint extends Command {

  private final Elevator elevator;
  private final double setpoint;
  private final SparkMax motor;
  private final PIDController pidController;
  

  /** Creates a new ElevatorGoToSetpoint. */
  public ElevatorGoToSetpoint(Elevator elevator, double setpoint) {

    this.elevator = elevator;
    this.setpoint = setpoint;

    this.motor = elevator.elevatorMotor;

    this.pidController = elevator.turnController;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = pidController.calculate(motor.getEncoder().getPosition(), setpoint);

    motor.set(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    motor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
