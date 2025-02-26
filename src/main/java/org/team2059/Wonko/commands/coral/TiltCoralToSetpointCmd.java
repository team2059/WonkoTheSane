package org.team2059.Wonko.commands.coral;

import org.littletonrobotics.junction.Logger;
import org.team2059.Wonko.Constants.CoralCollectorConstants;
import org.team2059.Wonko.subsystems.coral.CoralCollector;
import org.team2059.Wonko.util.LoggedTunableNumber;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;

public class TiltCoralToSetpointCmd extends Command {
  // Feedforward constants in the form of tunable numbers
  private LoggedTunableNumber kS = new LoggedTunableNumber("CoralCollector/Tilt/kS", CoralCollectorConstants.kSTilt);
  private LoggedTunableNumber kV = new LoggedTunableNumber("CoralCollector/Tilt/kV", CoralCollectorConstants.kVTilt);
  private LoggedTunableNumber kA = new LoggedTunableNumber("CoralCollector/Tilt/kA", CoralCollectorConstants.kATilt);
  private LoggedTunableNumber kG = new LoggedTunableNumber("CoralCollector/Tilt/kG", CoralCollectorConstants.kGTilt);

  // This profile holds constraints and allows you to get the next setpoint for motion profiling.
  private final TrapezoidProfile profile = new TrapezoidProfile(
    new TrapezoidProfile.Constraints(
      CoralCollectorConstants.tiltMaxVelocity, 
      CoralCollectorConstants.tiltMaxAccel
    )
  );

  // The goal is the final end state, the setpoint is just the next target.
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  private CoralCollector coralCollector;

  private double userGoal;

  private ArmFeedforward feedforward; // tried to use Arm... didn't work well.

  public TiltCoralToSetpointCmd(CoralCollector coralCollector, double userGoal) {
    this.coralCollector = coralCollector;
    this.userGoal = userGoal;
    feedforward = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get());

    // addRequirements(coralCollector);
  }

  @Override
  public void initialize() {
    // Set goal that was specified by the object creator (assume endpoint is stopped)
    goal = new TrapezoidProfile.State(userGoal, 0);

    // Set the setpoint as the current real position
    setpoint = new TrapezoidProfile.State(
      coralCollector.inputs.tiltMotorPositionRad, 
      coralCollector.inputs.tiltMotorVelocityRadPerSec
    );
  }

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
    coralCollector.io.setTiltPos(
      setpoint.position,
      feedforward.calculate(setpoint.position, setpoint.velocity)
    );


    Logger.recordOutput("SETPOINT TILT", setpoint.position);
  }

  @Override
  public void end(boolean interrupted) {
    // STOP the motor on end. Otherwise gravity will reset the mechanism and 
      // the next enable will cause rapid, undesired motion that might break something.
    coralCollector.io.setTiltVolts(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
