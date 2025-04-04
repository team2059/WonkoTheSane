// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2059.Wonko;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import org.team2059.Wonko.Constants.AlgaeCollectorConstants;
import org.team2059.Wonko.Constants.CoralCollectorConstants;
import org.team2059.Wonko.Constants.ElevatorConstants;
import org.team2059.Wonko.Constants.OperatorConstants;
import org.team2059.Wonko.Constants.VisionConstants;
import org.team2059.Wonko.commands.AutoCommands;
import org.team2059.Wonko.commands.ElevateToReefLevelCmd;
import org.team2059.Wonko.commands.drive.TeleopDriveCmd;
import org.team2059.Wonko.commands.drive.TeleopDriveCmdXbox;
import org.team2059.Wonko.commands.elevator.ElevateToSetpointCmd;
import org.team2059.Wonko.commands.vision.PathfindToHPS;
import org.team2059.Wonko.commands.vision.PathfindToReefCmd;
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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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

  SendableChooser<Command> autoChooser;

  public static Vision vision;
  public static Drivetrain drivetrain;
  public static Elevator elevator;
  public static AlgaeCollector algaeCollector;
  public static CoralCollector coralCollector;
  public static Climber climber;

  public static CommandXboxController xboxDriver;
  public static Joystick logitech;
  public static GenericHID buttonBox;
  public static XboxController xboxController;
  public static JoystickButton upperCamSwitch;
  public static JoystickButton lowerCamSwitch;

  public static Supplier<Double> strafe; 
  public static Supplier<Double> translation; 
  public static Supplier<Double> rotation; 

  public static boolean isRed = false; 

  public static boolean isSlowMode;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    isSlowMode = true;

    /* ========== */
    /* SUBSYSTEMS */
    /* ========== */

    vision = new Vision(new VisionIOReal());
    drivetrain = new Drivetrain(vision, new GyroIONavX());
    elevator = new Elevator(new ElevatorIOReal());
    algaeCollector = new AlgaeCollector(new AlgaeCollectorIOReal());
    coralCollector = new CoralCollector(new CoralCollectorIOReal());
    climber = new Climber(new ClimberIOReal());

    /* =========== */
    /* CONTROLLERS */
    /* =========== */
    
    xboxDriver = new CommandXboxController(OperatorConstants.xboxDriverPort);
    logitech = new Joystick(OperatorConstants.logitechPort);
    buttonBox = new GenericHID(OperatorConstants.buttonBoxPort);
    xboxController = new XboxController(OperatorConstants.xboxControllerPort);

    // Drive Controls
    if (OperatorConstants.useXboxForDriving) {
      translation = xboxDriver::getLeftX;
      strafe = xboxDriver::getLeftY;
      rotation = xboxDriver::getRightX;
    }

    /* ================ */
    /* DEFAULT COMMANDS */
    /* ================ */

    // Default commands run when the subsystem in question has no scheduled commands requiring it.
    if (OperatorConstants.useXboxForDriving) {
      drivetrain.setDefaultCommand(
      new TeleopDriveCmdXbox(
        drivetrain, 
        () -> -strafe.get(), // forwardX
        () -> -translation.get(), // forwardY
        () -> -rotation.get(), // rotation
        () -> isSlowMode
      )); 
    } else {
      drivetrain.setDefaultCommand(
      new TeleopDriveCmd(
        drivetrain, 
        () -> -logitech.getRawAxis(OperatorConstants.JoystickTranslationAxis), // forwardX
        () -> -logitech.getRawAxis(OperatorConstants.JoystickStrafeAxis), // forwardY
        () -> -logitech.getRawAxis(OperatorConstants.JoystickRotationAxis), // rotation
        () -> logitech.getRawAxis(OperatorConstants.JoystickSliderAxis), // slider
        () -> logitech.getRawButton(OperatorConstants.JoystickStrafeOnly), // Strafe Only Button
        () -> logitech.getRawButton(OperatorConstants.JoystickInvertedDrive) // Inverted buytton
      )
      );
    }
    
    elevator.setDefaultCommand(
      Commands.parallel(
        new ElevateToSetpointCmd(elevator, ElevatorConstants.levelHeights[0]),
        coralCollector.setTiltSetpointCmd(CoralCollectorConstants.levelCoralTiltAngle[0])
      ).until(() -> (elevator.inputs.zeroLimit || elevator.inputs.positionMeters <= 0.1))
    );

    algaeCollector.setDefaultCommand(
      algaeCollector.setTiltSetpointCmd(AlgaeCollectorConstants.thruBoreMaximum)
    );

    if (isRed) {
      VisionConstants.HPTags = VisionConstants.redHPTags; 
      VisionConstants.reefTags = VisionConstants.redReefTags; 
    } else {
      VisionConstants.HPTags = VisionConstants.blueHPTags; 
      VisionConstants.reefTags = VisionConstants.blueReefTags; 
    }

    /* ========== */
    /* AUTONOMOUS */
    /* ========== */

    // Register all NamedCommands
    AutoCommands.registerNamedCommands(drivetrain, coralCollector, algaeCollector, elevator, vision);

    // Build auto chooser - you can also set a default.
    autoChooser = AutoBuilder.buildAutoChooser();

    /* ======= */
    /* LOGGING */
    /* ======= */

    // Field for PathPlanner debugging
    var field = new Field2d();
    SmartDashboard.putData(field);
    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> { // current pose
      field.setRobotPose(pose);
    });
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> { // target pose
      field.getObject("target pose").setPose(pose);
    });
    PathPlannerLogging.setLogActivePathCallback((poses) -> { // active path (list of poses)
      field.getObject("trajectory").setPoses(poses);
    });

    // Build info
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

    // Allow viewing of command scheduler queue in dashboards 
    SmartDashboard.putData(CommandScheduler.getInstance());

    // Publish subsystem status to dashboard
    SmartDashboard.putData(vision);
    SmartDashboard.putData(drivetrain);
    SmartDashboard.putData(elevator);
    SmartDashboard.putData(algaeCollector);
    SmartDashboard.putData(coralCollector);
    SmartDashboard.putData(climber);

    // Publish auto chooser
    SmartDashboard.putData("Auto Chooser", autoChooser);
  
    upperCamSwitch = new JoystickButton(buttonBox, 13);
    lowerCamSwitch = new JoystickButton(buttonBox, 14);

    configureBindings();


  }
  public void slowMode() {
    if (isSlowMode) {
      isSlowMode = false;
    } else {
      isSlowMode = true;
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

    if (OperatorConstants.useXboxForDriving) {
      xboxDriver.start().onTrue(new InstantCommand(() -> drivetrain.zeroHeading()));
      xboxDriver.back().onTrue(new InstantCommand(() -> drivetrain.setFieldRelativity()));
      xboxDriver.rightBumper().whileTrue(new InstantCommand(() -> slowMode()));
      xboxDriver.leftTrigger().whileTrue(new PathfindToReefCmd(drivetrain, vision, false, true));
      xboxDriver.rightTrigger().whileTrue(new PathfindToReefCmd(drivetrain, vision, true, true));
    } else {
      /* RESET NAVX HEADING */
      new JoystickButton(logitech, OperatorConstants.JoystickResetHeading)
        .whileTrue(new InstantCommand(() -> drivetrain.zeroHeading()));

      /* SWITCH FIELD/ROBOT RELATIVITY IN TELEOP */
      new JoystickButton(logitech, OperatorConstants.JoystickRobotRelative)
        .whileTrue(new InstantCommand(() -> drivetrain.setFieldRelativity()));

      new JoystickButton(logitech, 12) // HP ALIGN
        .whileTrue(new PathfindToHPS(drivetrain, vision, true));
      new JoystickButton(logitech, 2) // LEFT REEF ALIGN
        .whileTrue(new PathfindToReefCmd(drivetrain, vision, false, false));
  
      new JoystickButton(logitech, 1) // RIGHT REEF ALIGN
        .whileTrue(new PathfindToReefCmd(drivetrain, vision, true, false));
    }

    /* ========== */
    /* Drivetrain */
    /* ========== */

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
    
    // Human player station (No longer needed due to funnel)
    new JoystickButton(buttonBox, 8)
      .whileTrue(Commands.parallel(
        new ElevateToSetpointCmd(elevator, ElevatorConstants.humanPlayerHeight),
        coralCollector.setTiltSetpointCmd(CoralCollectorConstants.humanPlayerAngle),
        coralCollector.intakeCommand()
      ));    

    // Processor
    new JoystickButton(buttonBox, 7)
      .whileTrue(
        Commands.parallel(
          algaeCollector.setTiltSetpointCmd(AlgaeCollectorConstants.thruBoreMinimum),
          new ElevateToSetpointCmd(elevator, ElevatorConstants.processorHeight)
        )
    );

    /* Toggle gyro 180 degree rotation */
    new JoystickButton(buttonBox, 15)
      .onTrue(new InstantCommand(() -> drivetrain.set180GyroRotation(true)))
      .onFalse(new InstantCommand(() -> drivetrain.set180GyroRotation(false)));

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
    
    // Intake/Outtake (on XboxController)
    new JoystickButton(xboxController, 2) // B
      .whileTrue(coralCollector.intakeCommand());
    new JoystickButton(xboxController, 1) // A
      .whileTrue(coralCollector.outtakeCommand());

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

    // Intake/Outtake (on XboxController)
    new JoystickButton(xboxController, 4) // Y
      .whileTrue(algaeCollector.intakeCommand());
    // new JoystickButton(xboxController, 3) // X
    //   .whileTrue(algaeCollector.outtakeCommand());

    // // Tilt up/down
    new JoystickButton(buttonBox, 5)
      .whileTrue(algaeCollector.setTiltSetpointCmd(AlgaeCollectorConstants.thruBoreMaximum));
      // .onFalse(new InstantCommand(() -> algaeCollector.io.stopTilt()));
    new JoystickButton(buttonBox, 6)
      .whileTrue(algaeCollector.setTiltSetpointCmd(AlgaeCollectorConstants.thruBoreMinimum));
      // .onFalse(new InstantCommand(() -> algaeCollector.io.stopTilt()));

    // Elevate & intake
    // Upper Algae
    new JoystickButton(buttonBox, 12).and(() -> !buttonBox.getRawButton(16))
      .whileTrue(new ParallelCommandGroup(
        new ElevateToSetpointCmd(elevator, Meters.of(AlgaeCollectorConstants.highAlgaeHeight)),
        algaeCollector.setTiltSetpointCmd(AlgaeCollectorConstants.thruBoreMinimum),
        algaeCollector.intakeCommand())
      );
    // Lower Algae
    new JoystickButton(buttonBox, 12).and(() -> buttonBox.getRawButton(16))
      .whileTrue(new ParallelCommandGroup(
        new ElevateToSetpointCmd(elevator, Meters.of(AlgaeCollectorConstants.lowAlgaeHeight)),
        algaeCollector.setTiltSetpointCmd(AlgaeCollectorConstants.thruBoreMinimum),
        algaeCollector.intakeCommand())
      );

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

    /* ====== */
    /* Vision */
    /* ====== */

    // new JoystickButton(logitech, 12) // HP ALIGN
    //   .whileTrue(new PathfindToHPS(drivetrain, vision));
    // new JoystickButton(logitech, 2) // LEFT REEF ALIGN
    //   .whileTrue(new PathfindToReefCmd(drivetrain, vision, false));

    // new JoystickButton(logitech, 1) // RIGHT REEF ALIGN
    //   .whileTrue(new PathfindToReefCmd(drivetrain, vision, true));

    // upperCamSwitch
    //   .onTrue(new InstantCommand(() -> vision.inputs.upperIsOn = false))
    //   .onFalse(new InstantCommand(() -> vision.inputs.upperIsOn = true)); 
    
    // lowerCamSwitch
    //   .onTrue(new InstantCommand(() -> vision.inputs.lowerIsOn = false))
    //   .onFalse(new InstantCommand(() -> vision.inputs.lowerIsOn = true)); 
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