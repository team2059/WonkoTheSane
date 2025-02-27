// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2059.Wonko;

import org.littletonrobotics.junction.Logger;
import org.team2059.Wonko.Constants.AlgaeCollectorConstants;
import org.team2059.Wonko.Constants.CoralCollectorConstants;
import org.team2059.Wonko.Constants.ElevatorConstants;
import org.team2059.Wonko.Constants.OperatorConstants;
import org.team2059.Wonko.commands.ElevateToReefLevelCmd;
import org.team2059.Wonko.commands.algae.TiltAlgaeToSetpointCommand;
import org.team2059.Wonko.commands.coral.TiltCoralToSetpointCmd;
import org.team2059.Wonko.commands.drive.TeleopDriveCmd;
import org.team2059.Wonko.commands.elevator.ElevateToSetpointCmd;
import org.team2059.Wonko.subsystems.algae.AlgaeCollector;
import org.team2059.Wonko.subsystems.algae.AlgaeCollectorIOReal;
import org.team2059.Wonko.subsystems.climber.Climber;
import org.team2059.Wonko.subsystems.climber.ClimberIOReal;
import org.team2059.Wonko.subsystems.coral.CoralCollector;
import org.team2059.Wonko.subsystems.coral.CoralCollectorIOReal;
import org.team2059.Wonko.subsystems.drive.Drivetrain;
import org.team2059.Wonko.subsystems.drive.GyroIONavX;
import org.team2059.Wonko.subsystems.elevator.Elevator;
import org.team2059.Wonko.subsystems.elevator.ElevatorIOReal;
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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
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
  public static Vision vision;
  public static Drivetrain drivetrain;
  public static Elevator elevator;
  public static AlgaeCollector algaeCollector;
  public static CoralCollector coralCollector;
  public static Climber climber;

  /* CONTROLLERS */
  public final static Joystick logitech = new Joystick(OperatorConstants.logitechPort);
  public final static GenericHID buttonBox = new GenericHID(OperatorConstants.buttonBoxPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    boolean isReal = RobotBase.isReal();

    // Subsystem creation
    vision = new Vision(isReal ? new VisionIOReal() : new VisionIOSim());
    drivetrain = new Drivetrain(vision, new GyroIONavX());
    elevator = new Elevator(new ElevatorIOReal());
    algaeCollector = new AlgaeCollector(new AlgaeCollectorIOReal());
    coralCollector = new CoralCollector(new CoralCollectorIOReal());
    climber = new Climber(new ClimberIOReal());

    // Build auto chooser and set default auto (you don't have to set a default)
    autoChooser = AutoBuilder.buildAutoChooser("New Auto");
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Allow viewing of command scheduler queue in dashboards 
    SmartDashboard.putData(CommandScheduler.getInstance());

    // Publish subsystem status to dashboard
    SmartDashboard.putData(vision);
    SmartDashboard.putData(drivetrain);
    SmartDashboard.putData(elevator);
    SmartDashboard.putData(algaeCollector);
    SmartDashboard.putData(coralCollector);
    SmartDashboard.putData(climber);

    /*
     * Send axes and buttons from joystick to TeleopDriveCmd,
     * which will govern the drivetrain during teleop
     */
    drivetrain.setDefaultCommand(new TeleopDriveCmd(
      drivetrain, 
      () -> -logitech.getRawAxis(OperatorConstants.JoystickTranslationAxis), // forwardX
      () -> -logitech.getRawAxis(OperatorConstants.JoystickStrafeAxis), // forwardY
      () -> -logitech.getRawAxis(OperatorConstants.JoystickRotationAxis), // rotation
      () -> logitech.getRawAxis(OperatorConstants.JoystickSliderAxis) // slider
    ));

    configureBindings();

    // Log build details to dashboard
    SmartDashboard.putString("ProjectName", "WonkoTheSane");
    SmartDashboard.putString("BuildDate", BuildConstants.BUILD_DATE);
    SmartDashboard.putString("GitSHA", BuildConstants.GIT_SHA);
    SmartDashboard.putString("GitDate", BuildConstants.GIT_DATE);
    SmartDashboard.putString("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        SmartDashboard.putString("GitDirty", "All changes committed");
        break;
      case 1:
        SmartDashboard.putString("GitDirty", "Uncommitted changes");
        break;
      default:
        SmartDashboard.putString("GitDirty", "Unknown");
        break;
    }
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

    /* ========== */
    /* Drivetrain */
    /* ========== */

    /* BUTTON 5: RESET NAVX HEADING */
    new JoystickButton(logitech, OperatorConstants.JoystickResetHeading)
      .whileTrue(new InstantCommand(() -> drivetrain.zeroHeading()));

    /* BUTTON 3: SWITCH FIELD/ROBOT RELATIVITY IN TELEOP */
    new JoystickButton(logitech, OperatorConstants.JoystickRobotRelative)
      .whileTrue(new InstantCommand(() -> drivetrain.setFieldRelativity()));
    
    // Drivetrain translation sysID routine (just drive motors) (wheels must be locked straight for this)
    // new JoystickButton(buttonBox, 1)
    //   .whileTrue(drivetrain.routine.quasistaticForward());
    // new JoystickButton(buttonBox, 2)
    //   .whileTrue(drivetrain.routine.quasistaticReverse());
    // new JoystickButton(buttonBox, 3)
    //   .whileTrue(drivetrain.routine.dynamicForward());
    // new JoystickButton(buttonBox, 4)
    //   .whileTrue(drivetrain.routine.dynamicReverse());

    /* ======== */
    /* Elevator */
    /* ======== */

    // Reef levels
    new JoystickButton(buttonBox, 1) // L1
      .whileTrue(new ElevateToReefLevelCmd(1, coralCollector, elevator));
    new JoystickButton(buttonBox, 2) // L2
      .whileTrue(new ElevateToReefLevelCmd(2, coralCollector, elevator));
    new JoystickButton(buttonBox, 3) // L3
      .whileTrue(new ElevateToReefLevelCmd(3, coralCollector, elevator));
    new JoystickButton(buttonBox, 4) // L4
      .whileTrue(new ElevateToReefLevelCmd(4, coralCollector, elevator));
    
    // Human player station
    new JoystickButton(buttonBox, 8)
      .whileTrue(Commands.parallel(
        new ElevateToSetpointCmd(elevator, ElevatorConstants.humanPlayerHeight),
        new TiltCoralToSetpointCmd(coralCollector, CoralCollectorConstants.humanPlayerAngle)
      ));    

    // Processor
    new JoystickButton(buttonBox, 7)
      .whileTrue(Commands.parallel(
        new ElevateToSetpointCmd(elevator, ElevatorConstants.processorHeight),
        new TiltAlgaeToSetpointCommand(algaeCollector, AlgaeCollectorConstants.thruBoreMinimum)
      ));

    // Elevator sysID routine
    // new JoystickButton(buttonBox, 5)
    //   .whileTrue(elevator.routine.quasistaticForward());
    // new JoystickButton(buttonBox, 6)
    //   .whileTrue(elevator.routine.quasistaticReverse());
    // new JoystickButton(buttonBox, 7)
    //   .whileTrue(elevator.routine.dynamicForward());
    // new JoystickButton(buttonBox, 8)
    //   .whileTrue(elevator.routine.dynamicReverse());

    /* =============== */
    /* Coral Collector */
    /* =============== */
    // Intake/Outtake (on Joystick)
    new JoystickButton(logitech, 9)
      .whileTrue(coralCollector.intakeCommand());
    new JoystickButton(logitech, 10)
      .whileTrue(coralCollector.outtakeCommand());

    // Resting/Idle Position
    new JoystickButton(buttonBox, 11)
      .whileTrue(new TiltCoralToSetpointCmd(coralCollector, CoralCollectorConstants.restingCoralCollectorPos));

    // Coral collector sysID routine
    // new JoystickButton(buttonBox, 1)
    //   .whileTrue(coralCollector.routine.quasistaticForward());
    // new JoystickButton(buttonBox, 2)
    //   .whileTrue(coralCollector.routine.quasistaticReverse());
    // new JoystickButton(buttonBox, 3)
    //   .whileTrue(coralCollector.routine.dynamicForward());
    // new JoystickButton(buttonBox, 4)
    //   .whileTrue(coralCollector.routine.dynamicReverse());

    /* =============== */
    /* Algae Collector */
    /* =============== */
    // Intake/Outtake (on Joystick)
    new JoystickButton(logitech, 11)
      .whileTrue(algaeCollector.intakeCommand());
    new JoystickButton(logitech, 12)
      .whileTrue(algaeCollector.outtakeCommand());

    // Tilt up/down
    new JoystickButton(buttonBox, 5)
      .whileTrue(new TiltAlgaeToSetpointCommand(algaeCollector, AlgaeCollectorConstants.thruBoreMaximum));
    new JoystickButton(buttonBox, 6)
      .whileTrue(new TiltAlgaeToSetpointCommand(algaeCollector, AlgaeCollectorConstants.thruBoreMinimum));

    // Algae sysID routine
    // new JoystickButton(buttonBox, 5)
    //   .whileTrue(algaeCollector.routine.quasistaticForward());
    // new JoystickButton(buttonBox, 6)
    //   .whileTrue(algaeCollector.routine.quasistaticReverse());
    // new JoystickButton(buttonBox, 7)
    //   .whileTrue(algaeCollector.routine.dynamicForward());
    // new JoystickButton(buttonBox, 8)
    //   .whileTrue(algaeCollector.routine.dynamicReverse());

    /* ======= */
    /* Climber */
    /* ======= */
    // Up/down
    new JoystickButton(buttonBox, 9)
      .whileTrue(climber.climberUpCommand());
    new JoystickButton(buttonBox, 10)
      .whileTrue(climber.climberDownCommand());

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
