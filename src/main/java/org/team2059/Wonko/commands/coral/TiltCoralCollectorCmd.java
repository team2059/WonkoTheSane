// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2059.Wonko.commands.coral;

import org.team2059.Wonko.subsystems.coral.CoralCollector;
import org.team2059.Wonko.util.LoggedTunableNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TiltCoralCollectorCmd extends Command {

  private CoralCollector coralCollector;
  private double setpoint;
  private PIDController controller;

  // Tunable numbers - can be changed on the fly from a dashboard
  private LoggedTunableNumber kP = new LoggedTunableNumber("TiltCoralCollectorCmd/kP", 0.005);
  private LoggedTunableNumber kI = new LoggedTunableNumber("TiltCoralCollectorCmd/kI", 0.0);
  private LoggedTunableNumber kD = new LoggedTunableNumber("TiltCoralCollectorCmd/kD", 0.0);

  /** Creates a new TiltCoralCollectorCmd. */
  public TiltCoralCollectorCmd(CoralCollector coralCollector, double setpoint) {
    this.coralCollector = coralCollector;
    this.setpoint = setpoint;

    // Create PID controller using specified constants
    controller = new PIDController(kP.get(), kI.get(), kD.get());

    // Use addRequirements() here to declare subsystem dependencies.
    // Ensures no two commands run on the coral collector at once
    addRequirements(coralCollector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Conditions and settings for the PID controller
    controller.reset();
    controller.setTolerance(0.02); // Allow 1 degree of error
    controller.setSetpoint(setpoint); // Specify our setpoint
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Update tunables if any have changed
    if (
      kP.hasChanged(hashCode())
      || kI.hasChanged(hashCode())
      || kD.hasChanged(hashCode())
    ) {
      controller.setPID(kP.get(), kI.get(), kD.get());
    }

    // Set the speed of the tilt motor based on the PID controller's calculated output
    // (clamped between -1 and 1 for safety, this is percent output)
    coralCollector.io.setTiltSpeed(
      MathUtil.clamp(
        controller.calculate(coralCollector.inputs.thruBorePositionDegrees), 
        -0.1, 
        0.1
      )
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralCollector.io.setTiltSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Returns true when we're at the setpoint to end the command
    // return controller.atSetpoint();
    return false;
  }
}
