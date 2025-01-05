// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCmds;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class ShootAtRPMsCmd extends Command {
  Shooter shooter;
  double desiredRPMs;
  double shooterVelocity;

  /** Creates a new ShootAtRPMsCmd. */
  public ShootAtRPMsCmd(Shooter shooter, double desiredRPMs) {
    this.shooter = shooter;
    this.desiredRPMs = desiredRPMs;
    // addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.recordOutput("desiredRPMs ", desiredRPMs);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    shooterVelocity = shooter.getVelocity();

    Logger.recordOutput("currentVelocity", shooter.getVelocity());
    Logger.recordOutput("desiredRPMs", desiredRPMs);

    // if (shooter.getVelocity() >= desiredRPMs - 50 &&
    // RobotContainer.logitech.getRawButton(6)) {
    // shooter.indexerMotor.set(-0.33);
    // }

    shooter.shooterUpperController.setReference(desiredRPMs,
        CANSparkMax.ControlType.kVelocity);
    shooter.shooterLowerController.setReference(desiredRPMs,
        CANSparkMax.ControlType.kVelocity);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    // shooter.indexerMotor.set(0);
    shooter.shooterUpperMotor.set(0);
    shooter.shooterLowerMotor.set(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;

  }
}
