// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2059.Wonko;

import static edu.wpi.first.units.Units.*;

import org.team2059.Wonko.Constants.AlgaeCollectorConstants;
import org.team2059.Wonko.Constants.CoralCollectorConstants;
import org.team2059.Wonko.Constants.ElevatorConstants;
import org.team2059.Wonko.Constants.OperatorConstants;
import org.team2059.Wonko.Constants.VisionConstants;
import org.team2059.Wonko.commands.ElevateToReefLevelCmd;
import org.team2059.Wonko.commands.algae.TiltAlgaeToSetpointCommand;
import org.team2059.Wonko.commands.coral.TiltCoralToSetpointCmd;
import org.team2059.Wonko.commands.drive.TeleopDriveCmd;
import org.team2059.Wonko.commands.elevator.ElevateToSetpointCmd;
import org.team2059.Wonko.commands.vision.PathfindReefOffset;
import org.team2059.Wonko.commands.vision.PathfindToAnyTagCmd;
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
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

  public static Joystick logitech;
  public static GenericHID buttonBox;
  public static boolean isRed = false; 

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
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

    logitech = new Joystick(OperatorConstants.logitechPort);
    buttonBox = new GenericHID(OperatorConstants.buttonBoxPort);

    /* ================ */
    /* DEFAULT COMMANDS */
    /* ================ */

    // Default commands run when the subsystem in question has no scheduled commands requiring it.
    
    drivetrain.setDefaultCommand(
      new TeleopDriveCmd(
        drivetrain, 
        () -> -logitech.getRawAxis(OperatorConstants.JoystickTranslationAxis), // forwardX
        () -> -logitech.getRawAxis(OperatorConstants.JoystickStrafeAxis), // forwardY
        () -> -logitech.getRawAxis(OperatorConstants.JoystickRotationAxis), // rotation
        () -> logitech.getRawAxis(OperatorConstants.JoystickSliderAxis) // slider
      )
    );

    elevator.setDefaultCommand(
      new ElevateToReefLevelCmd(0, coralCollector, elevator)
        .until(() -> elevator.inputs.zeroLimit)
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

    // Throw all NamedCommands here.

    // Note: ElevateToReefLevelCmd is itself a SequentialCommandGroup, 
    // which elevates to specified level 0-4 and tilts the collector 
    // once within 0.1 m (3.9 in) of the setpoint
    NamedCommands.registerCommand(
      "ScoreCoralL3",  // name as shown in PathPlanner GUI
      new SequentialCommandGroup(

        // Up sequence - ends when error is <= 1%
        new ElevateToReefLevelCmd(3, coralCollector, elevator)
          .withTimeout(4),
        
        // Deposit - holds elevator while running outtake
        new ParallelCommandGroup(
          new ElevateToReefLevelCmd(3, coralCollector, elevator),
          coralCollector.outtakeCommand()
        ).withTimeout(1),

        // Run elevator back to ground. Remains until auto over
        new ElevateToReefLevelCmd(0, coralCollector, elevator)
      )
    );

    NamedCommands.registerCommand(
      "ScoreCoralL4",  // name as shown in PathPlanner GUI
      new SequentialCommandGroup(

        // Up sequence - ends when error is <= 1%
        new ElevateToReefLevelCmd(4, coralCollector, elevator)
          .withTimeout(4),
        
        // Deposit - holds elevator while running outtake
        new ParallelCommandGroup(
          new ElevateToReefLevelCmd(4, coralCollector, elevator),
          coralCollector.outtakeCommand()
        ).withTimeout(0.5)
      )
    );

    NamedCommands.registerCommand(
      "GoToStartingReefLeft", 
      new PathfindToAnyTagCmd(drivetrain, vision, VisionConstants.reefTags.get(3), 16.5, -10.8)
    );

    NamedCommands.registerCommand(
      "GoToDriverReefCenter", 
      new PathfindToAnyTagCmd(drivetrain, vision, VisionConstants.reefTags.get(0), 20, 0)
    );

    NamedCommands.registerCommand(
      "ElevateAndCollectAlgae", 
      new ParallelCommandGroup(
        new ElevateToSetpointCmd(elevator, Meters.of(1.13)),
        algaeCollector.intakeCommand().withTimeout(1)
      )
    );

    NamedCommands.registerCommand(
      "GoToHPStation", 
      new PathfindToAnyTagCmd(drivetrain, vision, VisionConstants.HPTags.get(1), 14, 4)
    );

    NamedCommands.registerCommand(
      "ElevateAndIntakeCoral", 
      Commands.parallel(
        new ElevateToSetpointCmd(elevator, ElevatorConstants.humanPlayerHeight),
        coralCollector.intakeCommand()
      ).until(() -> coralCollector.inputs.hasCoral)
    );

    NamedCommands.registerCommand(
      "ElevateToHP", 
      new ElevateToSetpointCmd(elevator, ElevatorConstants.humanPlayerHeight)
    );

    NamedCommands.registerCommand(
      "TiltCoralToHP",
      new TiltCoralToSetpointCmd(coralCollector, CoralCollectorConstants.humanPlayerAngle)
        .until(() -> coralCollector.inputs.tiltMotorPositionRad >= 0.56 * .95 && coralCollector.inputs.tiltMotorPositionRad <= 0.56 * 1.05) 
    );

    NamedCommands.registerCommand(
      "TiltCoralToZero",
      new TiltCoralToSetpointCmd(coralCollector, CoralCollectorConstants.thruBoreMaxmimum)
    );

    NamedCommands.registerCommand(
      "IntakeCoral", 
      coralCollector.intakeCommand()
    );

    NamedCommands.registerCommand(
      "ElevatorZero", 
      new ElevateToSetpointCmd(elevator, ElevatorConstants.levelHeights[0]).withTimeout(3)
    );

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
        new TiltCoralToSetpointCmd(coralCollector, CoralCollectorConstants.humanPlayerAngle),
        coralCollector.intakeCommand()
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

    // Elevate & intake
    new JoystickButton(buttonBox, 12)
      .whileTrue(new ParallelCommandGroup(
        new ElevateToSetpointCmd(elevator, Meters.of(1.13)),
        algaeCollector.intakeCommand()
      )
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

    new JoystickButton(buttonBox, 15)
    .whileTrue(new InstantCommand(() -> OperatorConstants.right = true))
    .whileFalse(new InstantCommand(() -> OperatorConstants.right = false)); 

    new JoystickButton(logitech, OperatorConstants.goToHPStation) 
      .whileTrue(new PathfindToAnyTagCmd(drivetrain, vision, VisionConstants.HPTags.get(1), 15, 4));

    new JoystickButton(logitech, OperatorConstants.alignReef) 
      .whileTrue(new PathfindReefOffset(drivetrain, vision, VisionConstants.reefTags, OperatorConstants.right)); 

     

    // new JoystickButton(buttonBox, 12)
    //   .whileTrue(new RepeatCommand(new PathfindToTagCmd(drivetrain, vision, 20, 25)));
    // new JoystickButton(buttonBox, 12)
    //   .whileTrue(new RepeatCommand(new PathfindToAnyTagCmd(drivetrain, vision, 20, 21, 0)));
    // new JoystickButton(logitech, 8)
    //   .whileTrue(new RepeatCommand(new PathfindToAnyTagCmd(drivetrain, vision, 21, 40, 0)));

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
