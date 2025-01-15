// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveBase;

public class TeleopSwerveCmd extends Command {
  private final SwerveBase swerveSubsystem;
  private final DoubleSupplier forwardX, forwardY, rotation, slider;
  private final SlewRateLimiter xLimiter, yLimiter, rotLimiter;

  /** Creates a new TeleopLogitechExtreme3DSwerveCmd. */
  public TeleopSwerveCmd(SwerveBase swerveSubsystem, DoubleSupplier forwardX, DoubleSupplier forwardY, DoubleSupplier rotation, DoubleSupplier slider) {

    this.swerveSubsystem = swerveSubsystem;
    this.forwardX = forwardX;
    this.forwardY = forwardY;
    this.rotation = rotation;
    this.slider = slider;

    this.xLimiter = new SlewRateLimiter(SwerveConstants.maxAcceleration);
    this.yLimiter = new SlewRateLimiter(SwerveConstants.maxAcceleration);
    this.rotLimiter = new SlewRateLimiter(SwerveConstants.maxAngularAcceleration);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /** 
     * Units are given in meters/sec and radians/sec
     * Since joysticks give output from -1 to 1, we multiply outputs by the max speed
     * Otherwise, the max speed would be 1 m/s and 1 rad/s
     */

    // Get joystick input as x, y, and rotation
    double xSpeed = -forwardX.getAsDouble();
    double ySpeed = -forwardY.getAsDouble();
    double rot = -rotation.getAsDouble();

    // Apply deadband
    xSpeed = Math.abs(xSpeed) > 0.25 ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > 0.35 ? ySpeed : 0.0;
    rot = Math.abs(rot) > 0.4 ? rot : 0.0;

    // Make the driving smoother
    xSpeed = xLimiter.calculate(xSpeed) * SwerveConstants.kTeleDriveMaxSpeed;
    ySpeed = yLimiter.calculate(ySpeed) * SwerveConstants.kTeleDriveMaxSpeed;
    rot = rotLimiter.calculate(rot) * SwerveConstants.kTeleDriveMaxAngularSpeed;

    // Apply slider limit
    double sliderVal = (-slider.getAsDouble() + 1) / 2;
    sliderVal = sliderVal < 0.15 ? 0.15 : sliderVal;
    xSpeed *= sliderVal;
    ySpeed *= sliderVal;
    rot *= sliderVal;

    xSpeed = -MathUtil.applyDeadband(xSpeed, 0.1, 0.75);
    ySpeed = -MathUtil.applyDeadband(ySpeed, 0.1, 0.75);
    rot = -MathUtil.applyDeadband(rot, 0.3, 0.75);

    double[] log = {xSpeed, ySpeed, rot};
    Logger.recordOutput("TELEOP SWERVE CMD", log);

    swerveSubsystem.drive(
      xSpeed,
      ySpeed, 
      rot, 
      SwerveBase.fieldRelativeStatus
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
