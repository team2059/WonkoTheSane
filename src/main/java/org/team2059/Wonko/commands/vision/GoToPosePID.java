// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2059.Wonko.commands.vision;

import org.littletonrobotics.junction.Logger;
import org.team2059.Wonko.Constants.VisionConstants;
import org.team2059.Wonko.subsystems.drive.Drivetrain;
import org.team2059.Wonko.util.LoggedTunableNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

/** Command which allows us to "manually" drive using PID to do precise alignment. */

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToPosePID extends Command {

  private Drivetrain drivetrain;
  
  private double xTarget;
  private double yTarget;
  private double rotTarget;

  private double xOffsetInches;
  private double yOffsetInches;

  private PIDController xController;
  private PIDController yController;
  private PIDController thetaController;

  private LoggedTunableNumber pX = new LoggedTunableNumber("VisionAlign/pX", 2);
  private LoggedTunableNumber pY = new LoggedTunableNumber("VisionAlign/pY", 3);
  private LoggedTunableNumber pT = new LoggedTunableNumber("VisionAlign/pT", 2.5);

  /** Creates a new AlignReefCmd. */
  public GoToPosePID(
    Drivetrain drivetrain, 
    double xTarget, 
    double yTarget, 
    double rotTarget,
    double xOffsetInches,
    double yOffsetInches
  ) {
    this.drivetrain = drivetrain;

    this.xTarget = xTarget;
    this.yTarget = yTarget;
    this.rotTarget = rotTarget;

    this.xOffsetInches = xOffsetInches;
    this.yOffsetInches = yOffsetInches;

    xController = new PIDController(pX.get(), 0, 0);
    yController = new PIDController(pY.get(), 0, 0);
    thetaController = new PIDController(pT.get(), 0, 0);
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }
  public GoToPosePID(
    Drivetrain drivetrain, 
    int tagId,
    double xOffsetInches,
    double yOffsetInches
  ) {

    this.drivetrain = drivetrain;

    this.yOffsetInches = yOffsetInches;
    this.xOffsetInches = xOffsetInches;

    this.xTarget = VisionConstants.aprilTagFieldLayout.getTagPose(tagId).get().getX();
    this.yTarget = VisionConstants.aprilTagFieldLayout.getTagPose(tagId).get().getY();
    this.rotTarget = VisionConstants.aprilTagFieldLayout.getTagPose(tagId).get().getRotation().getZ();

    xController = new PIDController(pX.get(), 0, 0);
    yController = new PIDController(pY.get(), 0, 0);
    thetaController = new PIDController(pT.get(), 0, 0);
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    var goalPose = new Pose2d(
      xTarget, yTarget, Rotation2d.fromRadians(rotTarget)
    );

    // if (isRight) {
    //   goalPose = goalPose.transformBy(new Transform2d(Units.inchesToMeters(15), Units.inchesToMeters(6), Rotation2d.fromRadians(Math.PI)));
    // } else {
    //   goalPose = goalPose.transformBy(new Transform2d(Units.inchesToMeters(15), Units.inchesToMeters(-9), Rotation2d.fromRadians(Math.PI)));
    // }
    goalPose = goalPose.transformBy(new Transform2d(Units.inchesToMeters(xOffsetInches), Units.inchesToMeters(yOffsetInches), Rotation2d.fromRadians(Math.PI)));

    Logger.recordOutput("GOAL POSE", goalPose);

    xController.setSetpoint(goalPose.getX());
    yController.setSetpoint(goalPose.getY());
    thetaController.setSetpoint(goalPose.getRotation().getRadians());

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    xController.setTolerance(Units.inchesToMeters(1));
    yController.setTolerance(Units.inchesToMeters(1));
    thetaController.setTolerance(Units.degreesToRadians(5));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    LoggedTunableNumber.ifChanged(
      hashCode(), 
      () -> {
        xController.setP(pX.get());
        yController.setP(pY.get());
        thetaController.setP(pT.get());
      }, 
      pX, pY, pT
    );

    var drivetrainPose = drivetrain.getPose();

    double xSpeed = xController.calculate(drivetrainPose.getX());
    double ySpeed = yController.calculate(drivetrainPose.getY());
    double thetaSpeed = thetaController.calculate(drivetrainPose.getRotation().getRadians());

    Logger.recordOutput("xSpeed", xSpeed);
    Logger.recordOutput("ySpeed", ySpeed);
    Logger.recordOutput("thetaSpeed", thetaSpeed);

    drivetrain.drive(
      MathUtil.clamp(-xSpeed, -1, 1), 
      MathUtil.clamp(-ySpeed, -1, 1), 
      MathUtil.clamp(thetaSpeed, -5, 5), 
      true
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint());
  }
}
