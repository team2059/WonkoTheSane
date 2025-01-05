// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.VisionCmds;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveBase;
import frc.robot.subsystems.Vision;

public class ParallelTurnToAngleCmd extends Command {
  SwerveBase swerveBase;
  Vision vision;
  final double ANGULAR_P = 4;
  final double ANGULAR_D = 0.0;
  double yaw = 0;
  PIDController turnController = new PIDController(ANGULAR_P, 0.0, ANGULAR_D);
  double rotationSpeed;
  boolean hasTarget;
  double measurement;
  int counter = 0;

  /** Creates a new TurnToAngleCmd. */
  public ParallelTurnToAngleCmd(SwerveBase swerveBase, Vision vision) {
    this.swerveBase = swerveBase;
    this.vision = vision;
    addRequirements(swerveBase);
    // turnController.enableContinuousInput(-180, 180);
    // turnController.setTolerance(0.5, 2.5);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnController.setTolerance(0.01);
    turnController.enableContinuousInput(-Math.PI, Math.PI);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Vision-alignment mode
    // Query the latest result from PhotonVision
    var result = vision.getCamera().getLatestResult();

    hasTarget = result.hasTargets();

    if (hasTarget == false) {
      this.cancel();
    } else {
      var targets = result.targets;
      for (PhotonTrackedTarget target : targets) {
        if ((target.getFiducialId() == 4 || target.getFiducialId() == 5 || target.getFiducialId() == 6
            || target.getFiducialId() == 7) && target.getPoseAmbiguity() <= 0.2) {

          double yaw = target.getBestCameraToTarget().getRotation().getZ();
          measurement = yaw;
          SmartDashboard.putNumber("measurement", measurement);

          rotationSpeed = MathUtil.clamp(turnController.calculate(measurement, Math.PI), -1, 1);

          SmartDashboard.putNumber("rotationSpeed", rotationSpeed);

          swerveBase.drive(0, 0, rotationSpeed, true);

        }
      }

    }
    // swerveBase.getNavX().reset();
  }
  // measurement = swerveBase.getHeading().getDegrees();

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    rotationSpeed = 0;
    swerveBase.drive(0, 0, 0, true);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return turnController.atSetpoint();
  }
}