// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2059.Wonko.commands.drive;

import org.team2059.Wonko.subsystems.drive.Drivetrain;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PIDSwerve extends Command {

  final double maxLinearVelocityMetersPerSec = 1;
  final double maxAngularVelocityRadPerSec = 5.0;

  private Drivetrain drivetrain;
  private final Pose2d goalPose;
  private PPHolonomicDriveController driveController = new PPHolonomicDriveController(
    new PIDConstants(5.0, 0, 0), 
    new PIDConstants(5.0, 0, 0)
  );

  /** Creates a new PIDSwerve. */
  public PIDSwerve(
    Drivetrain drivetrain,
    Pose2d goalPose
  ) {

    this.drivetrain = drivetrain;
    this.goalPose = goalPose;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();

    goalState.pose = goalPose;

    ChassisSpeeds cs = driveController.calculateRobotRelativeSpeeds(drivetrain.getPose(), goalState);

    drivetrain.driveRobotRelative(
      new ChassisSpeeds(
        MathUtil.clamp(cs.vxMetersPerSecond, -maxLinearVelocityMetersPerSec, maxLinearVelocityMetersPerSec),
        MathUtil.clamp(cs.vyMetersPerSecond, -maxLinearVelocityMetersPerSec, maxLinearVelocityMetersPerSec),
        MathUtil.clamp(cs.omegaRadiansPerSecond, -maxAngularVelocityRadPerSec, maxAngularVelocityRadPerSec)
      )
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopAllMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (false);
  }
}
