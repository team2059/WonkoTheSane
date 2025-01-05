// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.CollectorConstants;
import frc.robot.Constants.ScoringPresets;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.PathfindToTagCmd;
import frc.robot.commands.ShooterAndCollectorIndexerCmd;
import frc.robot.commands.TeleopSwerveCmd;
import frc.robot.commands.AutoCmds.AutoIntakeNoteCmd;
import frc.robot.commands.ClimberCmds.ClimbDownCmd;
import frc.robot.commands.ClimberCmds.ClimbUpCmd;
import frc.robot.commands.CollectorCmds.IntakeNoteCmd;
import frc.robot.commands.CollectorCmds.TiltCollectorToCollectPosCmd;
import frc.robot.commands.CollectorCmds.TiltCollectorToSetpointONLYCLIMBCmd;
import frc.robot.commands.CollectorCmds.TiltCollectorToShooterCmd;
import frc.robot.commands.ScoringCmds.ScoreAmpCmd;
import frc.robot.commands.ScoringCmds.VisionShootCmd;
import frc.robot.commands.ShooterCmds.ElevateShooterToTrapCmd;
import frc.robot.commands.ShooterCmds.MoveShooterElevatorDownCmd;
import frc.robot.commands.ShooterCmds.MoveShooterElevatorToSetpointCmd;
import frc.robot.commands.ShooterCmds.MoveShooterElevatorUpCmd;
import frc.robot.commands.ShooterCmds.RunIndexerCmd;
import frc.robot.commands.ShooterCmds.ShootAtRPMsCmd;
import frc.robot.commands.ShooterCmds.TiltShooterToCollectorCmd;
import frc.robot.commands.ShooterCmds.TiltShooterToRestPosCmd;
import frc.robot.commands.ShooterCmds.TiltShooterToSetpointCmd;
import frc.robot.commands.VisionCmds.ParallelTurnToAngleCmd;
import frc.robot.commands.VisionCmds.TurnToAngleCmd;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.LEDStrip;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveBase;
import frc.robot.subsystems.Vision;

// import com.pathplanner.lib.*;
// import com.pathplanner.lib.commands.PPSwerveControllerCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        public static boolean isRed = false;
        public static SendableChooser<Boolean> allianceChooser = new SendableChooser<>();

        /* XBOX CONTROLLER */
        public static final XboxController controller = new XboxController(0);

        /* LOGITECH */
        public final static Joystick logitech = new Joystick(1);

        /* Button Box */
        public final static GenericHID buttonBox = new GenericHID(2);

        private final int kLogitechTranslationAxis = 1;
        private final int kLogitechStrafeAxis = 0;
        private final int kLogitechRotationAxis = 2;
        private final int kLogitechSliderAxis = 3;
        private final int kZeroGyro = 5;
        private final int kFieldOriented = 12;
        private final int kInverted = 6; // switch
        private final int kGoToTagButton = 1; // switch
         private final int kStrafeOnly = 2;
        private final int kSlowEverything = 3;

        private final JoystickButton zeroGyro = new JoystickButton(logitech, kZeroGyro);

        private final JoystickButton goToTag = new JoystickButton(logitech, kGoToTagButton);

        private final JoystickButton invertButton = new JoystickButton(logitech, kInverted);

        /* Driver Buttons */

        // private final JoystickButton alignWithTarget = new JoystickButton(driver,
        // XboxController.Button.kRightBumper.value);
        // private final JoystickButton autoBalance = new JoystickButton(driver,
        // XboxController.Button.kX.value);

        /* Subsystems */
        private static final SwerveBase swerveBase = new SwerveBase();

        private static final Collector collector = new Collector();

        private static final Climber climber = new Climber();

        private final static Shooter shooter = new Shooter();

        private static final Vision vision = new Vision();

        public static Pose2d speakerPose = new Pose2d();

        // private final PowerDistributionPanel powerDistributionPanel = new
        // PowerDistributionPanel();

        /* LED Strips */
        public final LEDStrip ledStrip = new LEDStrip(shooter, collector, 0, 35);

        SendableChooser<Command> autoChooser;

        boolean isbeinginverted = false;

        /* Commands */

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {

                var field = new Field2d();
                SmartDashboard.putData("Field", field);

                // Logging callback for current robot pose
                PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
                        // Do whatever you want with the pose here
                        field.setRobotPose(pose);
                });

                // Logging callback for target robot pose
                PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
                        // Do whatever you want with the pose here
                        field.getObject("target pose").setPose(pose);
                });

                // Logging callback for the active path, this is sent as a list of poses
                PathPlannerLogging.setLogActivePathCallback((poses) -> {
                        // Do whatever you want with the poses here
                        field.getObject("trajectory").setPoses(poses);
                });

                // NamedCommands.registerCommand("PathfindToTagCmd",
                // new PathfindToTagCmd(swerveBase, vision, 4, 78));

                NamedCommands.registerCommand("AutoIntakeNoteCmd",
                                new AutoIntakeNoteCmd(collector,
                                                shooter)
                                                .until(collector::isNotePresent)
                                                .andThen(new SequentialCommandGroup(
                                                                new ParallelCommandGroup(
                                                                                new TiltShooterToCollectorCmd(
                                                                                                shooter)
                                                                                                .withTimeout(1.25),
                                                                                new TiltCollectorToShooterCmd(
                                                                                                collector)),
                                                                new WaitCommand(0.5),
                                                                new ShooterAndCollectorIndexerCmd(
                                                                                collector,
                                                                                shooter)
                                                                                .until(shooter::isNotePresent))));

                // NamedCommands.registerCommand("AutoIntakeNoteCmd",
                // new ConditionalCommand(
                // new SequentialCommandGroup(new InstantCommand(() ->
                // collector.setRollerMotor(0)),
                // new ParallelCommandGroup(new TiltShooterToCollectorCmd(shooter),
                // new TiltCollectorToShooterCmd(collector)),
                // new WaitCommand(0.5),
                // new ShooterAndCollectorIndexerCmd(collector, shooter)),
                // new InstantCommand(() -> collector.setRollerMotor(0.33)), () ->
                // collector.isNotePresent()));

                NamedCommands.registerCommand("ShootSubwooferSpeakerCmd",
                                new ParallelCommandGroup(new TiltShooterToSetpointCmd(shooter,
                                                -61),
                                                new ShootAtRPMsCmd(shooter,
                                                                3000),
                                                new SequentialCommandGroup(new WaitCommand(1).andThen(
                                                                new RunIndexerCmd(shooter)
                                                                                .withTimeout(1))))
                                                .withTimeout(1.66));

                NamedCommands.registerCommand("ShootSubwooferSpeakerFARCmd",
                                new ParallelCommandGroup(new TiltShooterToSetpointCmd(shooter,
                                                -35),
                                                new ShootAtRPMsCmd(shooter,
                                                                3750),
                                                new SequentialCommandGroup(new WaitCommand(1).andThen(
                                                                new RunIndexerCmd(shooter)
                                                                                .withTimeout(1.5))))
                                                .withTimeout(1.66));

                NamedCommands.registerCommand("ShootFARSubwooferSpeakerRIGHTCmd",
                                new ParallelCommandGroup(new TiltShooterToSetpointCmd(shooter,
                                                -36),
                                                new ShootAtRPMsCmd(shooter,
                                                                4000),
                                                new SequentialCommandGroup(new WaitCommand(1).andThen(
                                                                new RunIndexerCmd(shooter)
                                                                                .withTimeout(1.5))))
                                                .withTimeout(1.66));

                NamedCommands.registerCommand("ShootFARSubwooferSpeakerLEFTCmd",
                                new ParallelCommandGroup(new TiltShooterToSetpointCmd(shooter,
                                                -34.5),
                                                new ShootAtRPMsCmd(shooter,
                                                                4000),
                                                new SequentialCommandGroup(new WaitCommand(1).andThen(
                                                                new RunIndexerCmd(shooter)
                                                                                .withTimeout(1.5))))
                                                .withTimeout(1.66));

                NamedCommands.registerCommand("ShooterTiltToCollectorPos",
                                new TiltShooterToSetpointCmd(shooter, ShooterConstants.alignToCollectorPos));

                // NamedCommands.registerCommand("FarShotCmd",
                // (new TiltShooterToSetpointCmd(shooter,
                // -50)
                // .andThen(
                // new ShootAtRPMsCmd(shooter,
                // 500))
                // .alongWith(new SequentialCommandGroup(new WaitCommand(1), new
                // RunIndexerCmd(shooter)))
                // .withTimeout(4)));

                NamedCommands.registerCommand("RunIndexerCmd", new RunIndexerCmd(shooter));
                NamedCommands.registerCommand("TiltCollectorToCollectPosCmd",
                                new TiltCollectorToCollectPosCmd(collector));

                // Configure the button bindings
                configureButtonBindings();

                // NamedCommands.registerCommand("arm", new PIDTiltArmCmd(tiltArm, 0.65));

                autoChooser = AutoBuilder.buildAutoChooser();
                allianceChooser.addOption("RED", true);
                allianceChooser.setDefaultOption("BLUE", false);

                SmartDashboard.putData("Auto Chooser", autoChooser);
                SmartDashboard.putData("Alliance Chooser", allianceChooser);

                // invertButton.toggleOnTrue(new InstantCommand(() -> isbeinginverted =
                // !isbeinginverted)); invert toggle button

                swerveBase.setDefaultCommand(new TeleopSwerveCmd(swerveBase,
                () -> logitech.getRawAxis(kLogitechTranslationAxis),
                () -> logitech.getRawAxis(kLogitechStrafeAxis),
                () -> logitech.getRawAxis(kLogitechRotationAxis),
                () -> logitech.getRawAxis(kLogitechSliderAxis),
                () -> !logitech.getRawButton(kFieldOriented),
                () -> logitech.getRawButton(kInverted), () -> logitech.getRawButton(kStrafeOnly),
                    () -> logitech.getRawButton(kSlowEverything)));

                // collector.setDefaultCommand(new MoveCollectorToSetpointCmd(collector,
                // CollectorConstants.collectorTiltAlignToShooterPos)
                // .onlyIf(() -> collector.isNotePresent()));

                // shooter.setDefaultCommand(new InstantCommand(() ->
                // shooter.indexerMotor.set(0)));
                // collector.setDefaultCommand(new InstantCommand(() ->
                // collector.rollerMotor.set(0)));

        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link GenericHID} or one of its subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), anxd then passing
         * it to a {@link
         * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
         */
        private void configureButtonBindings() {
                /* Driver Buttons */

                zeroGyro.onTrue(new InstantCommand(() -> swerveBase.getNavX().zeroYaw()));

                // red
                // goToTag.whileTrue(new PathfindToTagCmd(swerveBase, vision, 4, 40));

                // blueChange
                // goToTag.whileTrue(new PathfindToTagCmd(swerveBase, vision, 7, 40));

                // goToTag.whileTrue(swerveBase.pathFindToPose(isRed ?
                // // red
                // new Pose2d(14, 4.5, new Rotation2d(Units.degreesToRadians(-150)))

                // // blueChange
                // : new Pose2d(2.4, 4.5, new Rotation2d(Units.degreesToRadians(-30))),

                // new PathConstraints(
                // 2.5, 1.5,
                // Units.degreesToRadians(540), Units.degreesToRadians(720)),
                // 0).alongWith(
                // new ParallelCommandGroup(new TiltShooterToSetpointCmd(shooter,
                // -35),
                // new ShootAtRPMsCmd(shooter,
                // 3500))));

                goToTag.whileTrue(new TiltShooterToSetpointCmd(shooter,
                                -88));

                new JoystickButton(logitech, 2).whileTrue(new ParallelTurnToAngleCmd(swerveBase, vision));
                // goToTag.whileTrue(swerveBase.pathFindToPose(

                // // blueChange
                // new Pose2d(2.4, 4.5, new Rotation2d(Units.degreesToRadians(-30))),

                // new PathConstraints(
                // 2.5, 1.5,
                // Units.degreesToRadians(540), Units.degreesToRadians(720)),
                // 0).alongWith(
                // new ParallelCommandGroup(new TiltShooterToSetpointCmd(shooter,
                // -35),
                // new ShootAtRPMsCmd(shooter,
                // 3750))));

                /* SHOOT SUBWOOFER */
                new JoystickButton(buttonBox, 1)
                                .whileTrue(new TiltShooterToSetpointCmd(shooter,
                                                -62.5)
                                                .alongWith(
                                                                new ShootAtRPMsCmd(shooter,
                                                                                2500)));
                /* SHOOTER MID */
                new JoystickButton(buttonBox, 2)
                                .whileTrue(new TiltShooterToSetpointCmd(shooter,
                                                -40)
                                                .alongWith(
                                                                new ShootAtRPMsCmd(shooter,
                                                                                3250)));
                /* SHOOTER FAR */
                new JoystickButton(buttonBox, 3)
                                .whileTrue(new TiltShooterToSetpointCmd(shooter,
                                                -30)
                                                .alongWith(
                                                                new ShootAtRPMsCmd(shooter,
                                                                                4000)));

                /* ZERO SHOOTER TILT */
                new JoystickButton(buttonBox, 4)
                                .whileTrue(new TiltShooterToSetpointCmd(shooter,
                                                -88));

                /* CLIMBER TILT COLLECTOR TO CLIMB POS */
                new JoystickButton(buttonBox, 13).whileTrue(new TiltCollectorToSetpointONLYCLIMBCmd(collector,
                                ClimberConstants.COLLECTOR_POS_BEFORE_CLIMBING));

                /* CLIMBER WINCH UP */
                new JoystickButton(buttonBox, 8)
                                .whileTrue(new ClimbUpCmd(climber));

                /* CLIMBER WINCH DOWN */
                new JoystickButton(buttonBox, 7)
                                .whileTrue(new ClimbDownCmd(climber));

                /* RESET SHOOTER ELEVATOR TO HOME */
                new JoystickButton(buttonBox, 5)
                                .whileTrue(new ParallelCommandGroup(new MoveShooterElevatorDownCmd(shooter, 1),
                                                new TiltShooterToCollectorCmd(shooter)));

                /* AMP */
                new JoystickButton(buttonBox, 6)
                                .whileTrue(new ScoreAmpCmd(shooter));

                /* AUTO ALIGN VISION */
                new JoystickButton(buttonBox, 11)
                                .whileTrue(new TurnToAngleCmd(swerveBase, vision));

                /* SHOOTER ELEVATOR TO TRAP POS */
                // new JoystickButton(buttonBox, 10)
                // .whileTrue(new ElevateShooterToTrapCmd(shooter));

                /* SHOOTER TILT TO TRAP POS */
                // new JoystickButton(buttonBox, 11)
                // .whileTrue(new TiltShooterToSetpointCmd(shooter,
                // ClimberConstants.SHOOTER_TILT_TRAP_POS));

                /* AMP */
                // new JoystickButton(buttonBox, 6)
                // .whileTrue(new TiltShooterToSetpointCmd(shooter,
                // 40).alongWith(
                // new ShootAtRPMsCmd(shooter, 1000)));
                // new JoystickButton(buttonBox, 6)
                // .whileTrue(new MoveShooterElevatorToSetpointCmd(shooter, 30));

                /* SHOOTER EJECT TO TRAP */
                new JoystickButton(buttonBox, 12)
                                .whileTrue(new InstantCommand(() -> shooter.setIndexMotorSpeed(1)))
                                .whileFalse(new InstantCommand(() -> shooter.setIndexMotorSpeed(0)));

                /* A - INTAKE HOME */
                new JoystickButton(controller, 1)
                                .onTrue(new TiltCollectorToShooterCmd(collector));
                // new JoystickButton(controller, 4)
                // .whileTrue(new ElevateShooterToTrapCmd(shooter));

                /* Y - INTAKE COLLECT */
                new JoystickButton(controller, 4)
                                .onTrue(new TiltCollectorToCollectPosCmd(collector));

                /* X - INTAKE ROLLER */
                new JoystickButton(controller, 3).whileTrue(new IntakeNoteCmd(collector,
                                shooter));

                /* B - OUTTAKE ROLLER */
                new JoystickButton(controller, 2)
                                .whileTrue(new InstantCommand(() -> collector.setRollerMotor(-0.5)))
                                .whileFalse(new InstantCommand(() -> collector.setRollerMotor(0)));

                /* WINDOW BUTTON - AUTO ALIGN */
                new JoystickButton(controller, 7).whileTrue(new ScoreAmpCmd(shooter));
                new JoystickButton(controller, 8)
                                .whileTrue(new ParallelCommandGroup(new MoveShooterElevatorDownCmd(shooter, 1),
                                                new TiltShooterToCollectorCmd(shooter)));

                /* LEFT BUMPER - VISION SHOOT */
                new JoystickButton(controller, 5)
                                .whileTrue(new TurnToAngleCmd(swerveBase, vision)
                                                .alongWith(new VisionShootCmd(shooter, vision)));
                // new JoystickButton(controller, 5)
                // .whileTrue(new TurnToAngleCmd(swerveBase, vision));

                // new JoystickButton(controller, 5)
                // .whileTrue(new ShootAtRPMsCmd(shooter, 1000));

                // new JoystickButton(controller, 5)
                // .whileTrue(
                // new ShootAtRPMsSupplierCmd(shooter,
                // () -> MathUtil.clamp(-MathUtil.applyDeadband(logitech.getRawAxis(3) * 5000,
                // 0.25),
                // 0, 5000)));

                // new JoystickButton(controller, 5)
                // .whileTrue(
                // new TiltShooterToSetpointCmd(shooter,
                // -25)
                // .andThen(
                // new ShootAtRPMsCmd(shooter,
                // 5000)));

                /* RIGHT BUMPER - RUN SHOOTER INDEXER (FIRE NOTE) */
                new JoystickButton(controller, 6)
                                .whileTrue(
                                                new RunIndexerCmd(shooter));

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
                // return new TiltShooterToSetpointCmd(shooter,
                // -61)
                // .andThen(
                // new ShootAtRPMsCmd(shooter,
                // 4000))
                // .alongWith(new SequentialCommandGroup(new WaitCommand(2), new
                // RunIndexerCmd(shooter)))
                // .withTimeout(4);

        }

        public SwerveBase getSwerveBase() {
                return swerveBase;
        }

        public static Shooter getShooter() {
                return shooter;
        }

        public Vision getVision() {
                return vision;
        }

        public static Collector getCollector() {
                return collector;
        }

}
