// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCmds;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;

public class MoveShooterElevatorUpCmd extends Command {
  Shooter shooter;
  double setPoint;
  double kP, kD, kV, maxVel, maxAcc;
  ProfiledPIDController profiledPIDController;
  ElevatorFeedforward elevatorFeedforward;
  double currentPosition;
  PIDController pidController;

  /** Creates a new ElevateToAmpCmd. */
  public MoveShooterElevatorUpCmd(Shooter shooter, double setPoint) {
    this.shooter = shooter;
    this.setPoint = setPoint;

    // Constraints Coefficients
    maxVel = 250;
    maxAcc = 125;

    // PID coefficients
    kP = 0.66;
    kD = 0;
    kV = 0.05;

    profiledPIDController = new ProfiledPIDController(kP, 0, kD,
        new TrapezoidProfile.Constraints(maxVel, maxAcc));

    pidController = new PIDController(kP, 0, 0);

    profiledPIDController.setGoal(new State(setPoint, 0));

    elevatorFeedforward = new ElevatorFeedforward(0, 0, kV);

    //addRequirements(shooter);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    profiledPIDController.setTolerance(1);
    profiledPIDController.reset(shooter.elevatorMotor.getEncoder().getPosition());
    Logger.recordOutput("shooter elevator up setpoint", setPoint);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    currentPosition = shooter.elevatorMotor.getEncoder().getPosition();

    // goal = final target of mechanism
    double pidOutput = pidController.calculate(currentPosition,
        setPoint);

    double pidVelocitySetpoint = profiledPIDController.getSetpoint().velocity;
    // SmartDashboard.putNumber("desiredVelocity", pidVelocitySetpoint);

    double ffOutput = elevatorFeedforward.calculate(pidVelocitySetpoint);

    // SmartDashboard.putNumber("ffOutput", ffOutput);

    // m_motor.set(MathUtil.clamp(-new Joystick(0).getRawAxis(1), -0.25, 0.25));

    // if (shooter.isTopLimitReached() || currentPosition >=
    // ShooterConstants.TOP_LIMIT) {
    // System.out.println("stopped!!!!!");
    // shooter.elevatorMotor.set(0);
    // } else {
    shooter.elevatorMotor.setVoltage(pidOutput);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("UP ENDED!");
    shooter.elevatorMotor.set(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooter.elevatorMotor.getEncoder().getPosition() >= setPoint - 1.5
        || shooter.isTopLimitReached();
  }
}
