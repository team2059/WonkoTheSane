// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveBase;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurnToTag extends Command {
  private final SwerveBase swerveSubsystem;
  private final Vision vision;
  private final int tagID;
  private final PIDController turnController = new PIDController(4, 0, 0.3);
  private double rotationSpeed;

  /** Creates a new TurnToTag. */
  public TurnToTag(SwerveBase swerveSubsystem, Vision vision, int tagID) {
    this.swerveSubsystem = swerveSubsystem;
    this.vision = vision;
    this.tagID = tagID;
    this.rotationSpeed = 0;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnController.setTolerance(0.01);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    boolean b = vision.hasTargets;

    PhotonTrackedTarget t = vision.getCertainTarget(tagID);

    if (b && t != null) {
      double yaw = t.getYaw();

      rotationSpeed = -MathUtil.clamp(turnController.calculate(yaw, 0), -1, 1);

      Logger.recordOutput("y", yaw);

      swerveSubsystem.drive(0, 0, rotationSpeed, true);
    } else {
      this.cancel();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    rotationSpeed = 0;
    swerveSubsystem.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return turnController.atSetpoint();
  }
}
