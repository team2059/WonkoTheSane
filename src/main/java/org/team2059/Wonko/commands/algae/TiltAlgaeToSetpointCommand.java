// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2059.Wonko.commands.algae;

import org.littletonrobotics.junction.Logger;
import org.team2059.Wonko.subsystems.algae.AlgaeCollector;
import org.team2059.Wonko.util.LoggedTunableNumber;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TiltAlgaeToSetpointCommand extends Command {

  // Feedforward constants in the form of tunable numbers
  private LoggedTunableNumber kS = new LoggedTunableNumber("AlgaeCollector/Tilt/kS", 0.0);
  private LoggedTunableNumber kV = new LoggedTunableNumber("AlgaeCollector/Tilt/kV", 0.0);
  private LoggedTunableNumber kA = new LoggedTunableNumber("AlgaeCollector/Tilt/kA", 0.0);
  private LoggedTunableNumber kG = new LoggedTunableNumber("AlgaeCollector/Tilt/kG", 0.0);

  // This profile holds constraints and allows you to get the next setpoint for motion profiling.
  private final TrapezoidProfile profile = new TrapezoidProfile(
    new TrapezoidProfile.Constraints(
      7.0, 
      5.0
    )
  );

  // The goal is the final end state, the setpoint is just the next target.
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  private AlgaeCollector algaeCollector;

  private double userGoal;

  private ArmFeedforward feedforward;

  /** Creates a new TiltAlgaeToSetpointCommand. */
  public TiltAlgaeToSetpointCommand(AlgaeCollector algaeCollector, double userGoal) {
    this.algaeCollector = algaeCollector;
    this.userGoal = userGoal;
    feedforward = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get());
    
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(algaeCollector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set goal that was specified by the object creator (assume endpoint is stopped)
    goal = new TrapezoidProfile.State(userGoal, 0);

    // Set the setpoint as the current real position
    setpoint = new TrapezoidProfile.State(
      algaeCollector.inputs.integratedTiltPosRadians, 
      algaeCollector.inputs.integratedTiltVelRadPerSec
    );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Update tunables if changed
    LoggedTunableNumber.ifChanged(
      hashCode(),
      () -> {
        feedforward.setKv(kV.get());
        feedforward.setKa(kA.get());
        feedforward.setKs(kS.get());
        feedforward.setKg(kG.get());
      },
      kS, kA, kV, kG
    );

    // Retreieve setpoint for the next timestep
    // Moves towards the goal while obeying constraints
    setpoint = profile.calculate(0.02, setpoint, goal);

    // Send setpoint to spark PID controller
    algaeCollector.io.setTiltPos(
      setpoint.position,
      feedforward.calculate(setpoint.position, setpoint.velocity)
    );


    Logger.recordOutput("ALG set TILT", setpoint.position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
        // STOP the motor on end. Otherwise gravity will reset the mechanism and 
      // the next enable will cause rapid, undesired motion that might break something.
    algaeCollector.io.setTiltVolts(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
