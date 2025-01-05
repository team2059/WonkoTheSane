// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ScoringCmds;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ScoringPresets;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.ShooterCmds.MoveShooterElevatorUpCmd;
import frc.robot.commands.ShooterCmds.ShootAtRPMsCmd;
import frc.robot.commands.ShooterCmds.TiltShooterToSetpointCmd;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreAmpCmd extends SequentialCommandGroup {

  /** Creates a new ScoreAmpCmd. */
  public ScoreAmpCmd(Shooter shooter) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new MoveShooterElevatorUpCmd(shooter, ScoringPresets.AMP_SHOOTER_HEIGHT).alongWith(
        new TiltShooterToSetpointCmd(shooter, ScoringPresets.AMP_SHOOTER_TILT),
        new ShootAtRPMsCmd(shooter, ScoringPresets.AMP_SHOOTER_VELOCITY)));
  }
}
