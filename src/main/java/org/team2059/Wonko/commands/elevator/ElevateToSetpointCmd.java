// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2059.Wonko.commands.elevator;

import org.littletonrobotics.junction.Logger;
import org.team2059.Wonko.subsystems.elevator.Elevator;
import org.team2059.Wonko.util.LoggedTunableNumber;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevateToSetpointCmd extends Command {
  private LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kG", 0.0);
  private LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS", 0.0);
  private LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/kV", 0.0);
  private LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/kA", 0.0);

  private final TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(1, 0.3));
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  private Elevator elevator;

  private double userGoal;

  private ElevatorFeedforward feedforward;

  /** Creates a new ElevateCmdSparkClosedLoop. */
  public ElevateToSetpointCmd(Elevator elevator, double userGoal) {

    this.elevator = elevator;
    this.userGoal = userGoal;
    feedforward = new ElevatorFeedforward(kS.get(), kG.get(), kV.get(), kA.get());

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    goal = new TrapezoidProfile.State(userGoal, 0);
    setpoint = new TrapezoidProfile.State();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Update tunables
    if (
      kS.hasChanged(hashCode())
      || kG.hasChanged(hashCode())
      || kV.hasChanged(hashCode())
      || kA.hasChanged(hashCode())
    ) {
      feedforward.setKs(kS.get());
      feedforward.setKg(kG.get());
      feedforward.setKv(kV.get());
      feedforward.setKa(kA.get());
    }
  
    // Retrieve profiled setpoint for the next timestep. 
    // This setpoint moves toward the goal while obeying the constraints.
    setpoint = profile.calculate(0.02, setpoint, goal);

    // Send setpoint to offboard controller PID
    elevator.io.setPositionClosedLoopWithFF(
      setpoint.position, 
      feedforward.calculate(setpoint.velocity)
    );

    Logger.recordOutput("Desired motion", setpoint.position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.io.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
