// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CollectorCmds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ShooterCmds.TiltShooterToCollectorCmd;
import frc.robot.commands.ShooterAndCollectorIndexerCmd;

import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Shooter;

public class IntakeNoteCmd extends Command {

  Collector collector;
  Shooter shooter;

  boolean isReady;
  // boolean isFinished = false;

  /** Creates a new CollectCmd. */
  public IntakeNoteCmd(Collector collector, Shooter shooter) {
    this.collector = collector;
    this.shooter = shooter;
    addRequirements(collector, shooter);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (collector.isNotePresent()) {
      // isReady = true;
      // if (isReady) {
      // collector.rollerMotor.set(0);
      // }

      CommandScheduler.getInstance()
          .schedule(new SequentialCommandGroup(
              new ParallelCommandGroup(new TiltShooterToCollectorCmd(shooter).withTimeout(1),
                  new TiltCollectorToShooterCmd(collector)),
              new WaitCommand(0.25),
              new ShooterAndCollectorIndexerCmd(collector, shooter)));

      // isReady = false;

      // runIndexers();
      // isFinished = true;
      // CommandScheduler.getInstance()
      // .schedule(new SequentialCommandGroup(new ParallelCommandGroup(new
      // FeedNoteToShooterCmd(collector),
      // new TiltShooterToCollectorCmd(shooter)),
      // new ParallelCommandGroup(
      // new InstantCommand(() -> shooter.setIndexMotorSpeed(-0.33)),
      // new InstantCommand(() -> collector.rollerMotor.set(-0.25)))
      // .onlyWhile(() -> !shooter.isNotePresent()),
      // new InstantCommand(() -> shooter.setIndexMotorSpeed(0)),
      // new InstantCommand(() -> collector.rollerMotor.set(0))));

      // CommandScheduler.getInstance()
      // .schedule(new SequentialCommandGroup(new ParallelCommandGroup(new
      // FeedNoteToShooterCmd(collector),
      // new TiltShooterToCollectorCmd(shooter)),
      // new ParallelCommandGroup(
      // new InstantCommand(() -> shooter.setIndexMotorSpeed(-0.33)),
      // new InstantCommand(() -> collector.rollerMotor.set(-0.25))))
      // .until(() -> shooter.isNotePresent()));
      // if (shooter.isNotePresent()) {
      // shooter.setIndexMotorSpeed(0);
      // collector.rollerMotor.set(0);
      // }

    } else {
      collector.setRollerMotor(0.4);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    collector.rollerMotor.set(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // System.out.println(isFinished);

    return false;

  }
}
