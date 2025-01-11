// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.TeleopSwerveCmd;
import frc.robot.subsystems.SwerveBase;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  /* SENDABLES */
  SendableChooser<Command> autoChooser;

  /* SUBSYSTEMS */
  private static final SwerveBase swerveSubsystem = new SwerveBase();

  /* CONTROLLERS */
  public final static Joystick logitech = new Joystick(OperatorConstants.LogitechControllerPort);
  //public final static XboxController xboxController = new XboxController(OperatorConstants.XboxControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Builds auto chooser and sets default auto (you don't have to set a default)
    autoChooser = AutoBuilder.buildAutoChooser("New Auto");

    /*
     * Send axes and buttons from joystick to TeleopSwerveCmd,
     * which will govern the SwerveSubsystem during teleop
     */
    swerveSubsystem.setDefaultCommand(new TeleopSwerveCmd(
      swerveSubsystem, 
      () -> logitech.getRawAxis(1), // forwardX
      () -> logitech.getRawAxis(0), // forwardY
      () -> logitech.getRawAxis(2), // rotation
      () -> logitech.getRawAxis(3) // slider
    ));

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    /* BUTTON 5: RESET NAVX HEADING */
    new JoystickButton(logitech, 5)
      .whileTrue(new InstantCommand(() -> swerveSubsystem.getNavX().zeroYaw()));

    /* BUTTON 3: SWITCH FIELD/ROBOT RELATIVITY IN TELEOP */
    new JoystickButton(logitech, 3).whileTrue(new InstantCommand(() -> swerveSubsystem.setFieldRelativity()));

  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
