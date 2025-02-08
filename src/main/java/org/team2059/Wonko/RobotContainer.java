// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2059.Wonko;

import org.team2059.Wonko.Constants.OperatorConstants;
import org.team2059.Wonko.commands.TeleopDriveCmd;
import org.team2059.Wonko.subsystems.AlgaeIntake;
import org.team2059.Wonko.subsystems.CoralIntake;
import org.team2059.Wonko.subsystems.drive.Drivetrain;
import org.team2059.Wonko.subsystems.drive.GyroIONavX;
import org.team2059.Wonko.subsystems.vision.Vision;
import org.team2059.Wonko.subsystems.vision.VisionIOReal;
import org.team2059.Wonko.subsystems.vision.VisionIOSim;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  private static Vision vision;
  public static Drivetrain drivetrain;
  // private static final CoralIntake coralIntake = new CoralIntake();
  // private static final AlgaeIntake algaeIntake = new AlgaeIntake();

  /* CONTROLLERS */
  public final static Joystick logitech = new Joystick(OperatorConstants.logitechControllerPort);
  public final static GenericHID buttonBox = new GenericHID(OperatorConstants.buttonBoxPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    vision = new Vision(RobotBase.isReal() ? new VisionIOReal() : new VisionIOSim());
    drivetrain = new Drivetrain(
      vision,
      new GyroIONavX()
    );

    // Builds auto chooser and sets default auto (you don't have to set a default)
    autoChooser = AutoBuilder.buildAutoChooser("New Auto");
    SmartDashboard.putData("Auto Chooser", autoChooser);

    /*
     * Send axes and buttons from joystick to TeleopSwerveCmd,
     * which will govern the drivetrain during teleop
     */
    drivetrain.setDefaultCommand(new TeleopDriveCmd(
      drivetrain, 
      () -> -logitech.getRawAxis(1), // forwardX
      () -> -logitech.getRawAxis(0), // forwardY
      () -> -logitech.getRawAxis(2), // rotation
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
    new JoystickButton(logitech, OperatorConstants.JoystickResetHeading)
      .whileTrue(new InstantCommand(() -> drivetrain.zeroHeading()));

    /* BUTTON 3: SWITCH FIELD/ROBOT RELATIVITY IN TELEOP */
    new JoystickButton(logitech, OperatorConstants.JoystickRobotRelative)
      .whileTrue(new InstantCommand(() -> drivetrain.setFieldRelativity()));

    // /* INTAKE CORAL */
    // new JoystickButton(logitech, OperatorConstants.JoystickIntakeCoral)
    //   .whileTrue(new InstantCommand(() -> coralIntake.setIntakeSpeed(-0.1)))
    //   .whileFalse(new InstantCommand(() -> coralIntake.setIntakeSpeed(0)));

    // /* RELEASE CORAL */
    // new JoystickButton(logitech, OperatorConstants.JoystickReleaseCoral)
    //   .whileTrue(new InstantCommand(() -> coralIntake.setIntakeSpeed(0.5)))
    //   .whileFalse(new InstantCommand(() -> coralIntake.setIntakeSpeed(0)));
    
    // /* INTAKE ALGAE */
    // new JoystickButton(logitech, OperatorConstants.JoystickIntakeAlgae)
    //   .whileTrue(new InstantCommand(() -> algaeIntake.setEndEffectorSpeed(0.25)))
    //   .whileFalse(new InstantCommand(() -> algaeIntake.setEndEffectorSpeed(0))); 

    // /* RELEASE ALGAE */
    // new JoystickButton(logitech, OperatorConstants.JoystickReleaseAlgae)
    //   .whileTrue(new InstantCommand(() -> algaeIntake.setEndEffectorSpeed(-0.25)))
    //   .whileFalse(new InstantCommand(() -> algaeIntake.setEndEffectorSpeed(0))); 

    // new JoystickButton(logitech, 8)
    //   .whileTrue(new TurnParallelToTag(drivetrain, vision, 4));
    
    // Bind full set of SysId routine tests to buttons; a complete routine should run each of these once.
    // new JoystickButton(buttonBox, 1)
    //   .whileTrue(drivetrain.drivetrainRoutine.quasistaticForward());

    // new JoystickButton(buttonBox, 2)
    //   .whileTrue(drivetrain.drivetrainRoutine.quasistaticReverse());

    // new JoystickButton(buttonBox, 3)
    //   .whileTrue(drivetrain.drivetrainRoutine.dynamicForward());
      
    // new JoystickButton(buttonBox, 4)
    //   .whileTrue(drivetrain.drivetrainRoutine.dynamicReverse());
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
