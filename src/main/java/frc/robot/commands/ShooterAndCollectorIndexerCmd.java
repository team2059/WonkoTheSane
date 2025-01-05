// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Shooter;

public class ShooterAndCollectorIndexerCmd extends Command {
  Shooter shooter;
  Collector collector;

  /** Creates a new IndexerCmd. */
  public ShooterAndCollectorIndexerCmd(Collector collector, Shooter shooter) {
    this.shooter = shooter;
    this.collector = collector;
    addRequirements(shooter, collector);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!shooter.isNotePresent()) {
      shooter.setIndexMotorSpeed(-0.5);
      collector.setRollerMotor(-0.4);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setIndexMotorSpeed(0);
    collector.setRollerMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooter.isNotePresent();
  }
}
