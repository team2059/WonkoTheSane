// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ScoringCmds;

import java.util.Set;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShooterCmds.ShootAtRPMsCmd;
import frc.robot.commands.ShooterCmds.TiltShooterToSetpointCmd;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class VisionShootCmd extends SequentialCommandGroup {
  Shooter shooter;
  Vision vision;
  double distanceToSpeaker;
  double[] desiredShooterState;
  double desiredShooterVelocity;
  double desiredShooterAngle;

  /** Creates a new VisionShootSequentialCmd. */
  public VisionShootCmd(Shooter shooter, Vision vision) {
    this.shooter = shooter;
    this.vision = vision;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // addCommands(new ProxyCommand(() -> getCommand()));
    addCommands(Commands.defer(() -> getCommand(), Set.of(shooter, vision)));
  }

  public Command getCommand() {

    distanceToSpeaker = vision.distanceToSpeakerFieldToCamera;

    desiredShooterState = shooter.calculateDesiredShooterState(distanceToSpeaker);
    desiredShooterVelocity = desiredShooterState[0];
    desiredShooterAngle = desiredShooterState[1];

    SmartDashboard.putNumber("desiredShooterVelocity", desiredShooterVelocity);
    SmartDashboard.putNumber("desiredShooterAngle", desiredShooterAngle);

    return new TiltShooterToSetpointCmd(shooter,
        desiredShooterState[1])
        .alongWith(
            new ShootAtRPMsCmd(shooter,
                desiredShooterState[0]));

  }

}
