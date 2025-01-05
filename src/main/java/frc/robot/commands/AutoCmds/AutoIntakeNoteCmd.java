// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Shooter;

public class AutoIntakeNoteCmd extends Command {

  Collector collector;
  Shooter shooter;

  boolean isReady;
  // boolean isFinished = false;

  /** Creates a new CollectCmd. */
  public AutoIntakeNoteCmd(Collector collector, Shooter shooter) {
    this.collector = collector;
    this.shooter = shooter;
    // addRequirements(collector, shooter);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {

    collector.setRollerMotor(0.4);

    // if (collector.isNotePresent()) {

    // CommandScheduler.getInstance()
    // .schedule(new SequentialCommandGroup(new ParallelCommandGroup(new
    // TiltShooterToCollectorCmd(shooter),
    // new TiltCollectorToShooterCmd(collector)), new WaitCommand(0.5),
    // new ShooterAndCollectorIndexerCmd(collector, shooter)));

    // } else {
    // collector.setRollerMotor(0.55);
    // }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    collector.rollerMotor.set(0);

    // CommandScheduler.getInstance().schedule(new TiltShooterToSetpointCmd(shooter,
    // -40)
    // .andThen(
    // new ShootAtRPMsCmd(shooter,
    // 4000))
    // .alongWith(new SequentialCommandGroup(new WaitCommand(1), new
    // RunIndexerCmd(shooter)))
    // .withTimeout(4));

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // System.out.println(isFinished);

    return false;

  }
}
