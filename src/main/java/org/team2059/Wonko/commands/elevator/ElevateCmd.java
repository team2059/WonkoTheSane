// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2059.Wonko.commands.elevator;

import org.littletonrobotics.junction.Logger;
import org.team2059.Wonko.subsystems.elevator.Elevator;
import org.team2059.Wonko.util.LoggedTunableNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevateCmd extends Command {
  private Elevator elevator;
  private double goal;
  
  // Create tunable numbers, these are editable via AdvantageScope or a dashboard
  private LoggedTunableNumber kP = new LoggedTunableNumber("ElevateCmd/kP", 0.0);
  private LoggedTunableNumber kD = new LoggedTunableNumber("ElevateCmd/kD", 0.0);
  private LoggedTunableNumber kS = new LoggedTunableNumber("ElevateCmd/kS", 0.0);
  private LoggedTunableNumber kG = new LoggedTunableNumber("ElevateCmd/kG", 0.0);
  private LoggedTunableNumber kV = new LoggedTunableNumber("ElevateCmd/kV", 0.0);
  private LoggedTunableNumber kA = new LoggedTunableNumber("ElevateCmd/kA", 0.0);

  private ProfiledPIDController controller;
  private ElevatorFeedforward feedforward;
  private double currentPosition;

  /** Creates a new ElevateCmd. */
  public ElevateCmd(Elevator elevator, double goal) {

    this.elevator = elevator;
    this.goal = goal;

    // A profiled PID controller is essentially a PID controller that is motion profiled (has max velocity & acceleration constraints)
    controller = new ProfiledPIDController(
      kP.get(), 
      0.0, 
      kD.get(), 
      new TrapezoidProfile.Constraints(0.3, 0.3)
    );

    // Set the specified goal (this is the target endpoint)
    // Since this is position control, we want to be stopped (velocity 0) at the goal height
    controller.setGoal(new State(goal, 0));
    
    // Create a feedforward model, this gives a best estimate for required voltage, must be well tuned
    feedforward = new ElevatorFeedforward(kS.get(), kG.get(), kV.get(), kA.get());

    // This prevents two commands from running on the same subsytem at the same time
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set a certain tolerable error value
    controller.setTolerance(0.01);

    // Reset the controller as the first visible state
    controller.reset(elevator.inputs.positionMeters);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Check if tunable values have changed and set them appropriately
    if (
      kP.hasChanged(hashCode())
      || kD.hasChanged(hashCode())
      || kS.hasChanged(hashCode())
      || kG.hasChanged(hashCode())
      || kV.hasChanged(hashCode())
      || kA.hasChanged(hashCode())
    ) {
      controller.setPID(kP.get(), 0.0, kD.get());

      feedforward.setKs(kS.get());
      feedforward.setKg(kG.get());
      feedforward.setKv(kV.get());
      feedforward.setKa(kA.get());
    }

    // Record current position of elevator
    currentPosition = elevator.inputs.positionMeters;

    double setpointVelocity = controller.getSetpoint().velocity;

    // Get output for next setpoint state
    // PID output is the required output to the next setpoint
    double pidOutput = controller.calculate(currentPosition, goal);
    // FF output is the best-estimate output for the target velocity
    double ffOutput = feedforward.calculate(setpointVelocity);

    // Plumb voltage to motor, making sure output is between [-12, 12]
    elevator.io.setVoltage(MathUtil.clamp(pidOutput + ffOutput, -12.0, 12.0));

    Logger.recordOutput("Desired motion", controller.getSetpoint().position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.io.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // If the controller says that we are within the tolerance we specified in initialize(),
    // return true to stop the command.
    // return controller.atGoal();
    return false;
  }
}
