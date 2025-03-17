package org.team2059.Wonko.commands;

import org.team2059.Wonko.Constants.CoralCollectorConstants;
import org.team2059.Wonko.Constants.ElevatorConstants;
import org.team2059.Wonko.Constants.VisionConstants;
import org.team2059.Wonko.commands.elevator.ElevateToSetpointCmd;
import org.team2059.Wonko.commands.vision.GoToPosePID;
import org.team2059.Wonko.commands.vision.PathfindToHPS;
import org.team2059.Wonko.commands.vision.PathfindToReefCmd;
import org.team2059.Wonko.subsystems.algae.AlgaeCollector;
import org.team2059.Wonko.subsystems.coral.CoralCollector;
import org.team2059.Wonko.subsystems.drive.Drivetrain;
import org.team2059.Wonko.subsystems.elevator.Elevator;
import org.team2059.Wonko.subsystems.vision.Vision;
import org.team2059.Wonko.util.Elastic;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

/** 
 * Organization class for registering all
 * NamedCommands to be referenced in auto.
 */
public final class AutoCommands {
    public static void registerNamedCommands(
        Drivetrain drivetrain,
        CoralCollector coralCollector,
        AlgaeCollector algaeCollector,
        Elevator elevator,
        Vision vision
    ) {

        // GO TO REEF TAG PID
        final double goToReefTagTimeout = 2;
        NamedCommands.registerCommand(
            "Tag20Right", 
            new GoToPosePID(
                drivetrain, 
                20, 
                VisionConstants.reefXOffsetInches,
                VisionConstants.reefYRightOffsetInches
            ).withTimeout(goToReefTagTimeout)
        );
        NamedCommands.registerCommand(
            "Tag20Left", 
            new GoToPosePID(
                drivetrain, 
                20, 
                VisionConstants.reefXOffsetInches,
                VisionConstants.reefYLeftOffsetInches
            ).withTimeout(goToReefTagTimeout)
        );
        NamedCommands.registerCommand(
            "Tag21Right", 
            new GoToPosePID(
                drivetrain, 
                21, 
                VisionConstants.reefXOffsetInches,
                VisionConstants.reefYRightOffsetInches
            ).withTimeout(goToReefTagTimeout)
        );
        NamedCommands.registerCommand(
            "Tag21Left", 
            new GoToPosePID(
                drivetrain, 
                21, 
                VisionConstants.reefXOffsetInches,
                VisionConstants.reefYLeftOffsetInches
            ).withTimeout(goToReefTagTimeout)
        );
        NamedCommands.registerCommand(
            "Tag22Right", 
            new GoToPosePID(
                drivetrain, 
                22, 
                VisionConstants.reefXOffsetInches,
                VisionConstants.reefYRightOffsetInches
            ).withTimeout(goToReefTagTimeout)
        );
        NamedCommands.registerCommand(
            "Tag22Left", 
            new GoToPosePID(
                drivetrain, 
                22, 
                VisionConstants.reefXOffsetInches,
                VisionConstants.reefYLeftOffsetInches
            ).withTimeout(goToReefTagTimeout)
        );
        NamedCommands.registerCommand(
            "Tag9Right", 
            new GoToPosePID(
                drivetrain, 
                9, 
                VisionConstants.reefXOffsetInches,
                VisionConstants.reefYRightOffsetInches
            ).withTimeout(goToReefTagTimeout)
        );
        NamedCommands.registerCommand(
            "Tag9Left", 
            new GoToPosePID(
                drivetrain, 
                9, 
                VisionConstants.reefXOffsetInches,
                VisionConstants.reefYLeftOffsetInches
            ).withTimeout(goToReefTagTimeout)
        );
        NamedCommands.registerCommand(
            "Tag10Right", 
            new GoToPosePID(
                drivetrain, 
                10, 
                VisionConstants.reefXOffsetInches,
                VisionConstants.reefYRightOffsetInches
            ).withTimeout(goToReefTagTimeout)
        );
        NamedCommands.registerCommand(
            "Tag10Left", 
            new GoToPosePID(
                drivetrain, 
                10, 
                VisionConstants.reefXOffsetInches,
                VisionConstants.reefYLeftOffsetInches
            ).withTimeout(goToReefTagTimeout)
        );
        NamedCommands.registerCommand(
            "Tag11Right", 
            new GoToPosePID(
                drivetrain, 
                11, 
                VisionConstants.reefXOffsetInches,
                VisionConstants.reefYRightOffsetInches
            ).withTimeout(goToReefTagTimeout)
        );
        NamedCommands.registerCommand(
            "Tag11Left", 
            new GoToPosePID(
                drivetrain, 
                11, 
                VisionConstants.reefXOffsetInches,
                VisionConstants.reefYLeftOffsetInches
            ).withTimeout(goToReefTagTimeout)
        );
        NamedCommands.registerCommand(
            "Tag17Right", 
            new GoToPosePID(
                drivetrain, 
                17, 
                VisionConstants.reefXOffsetInches, 
                VisionConstants.reefYRightOffsetInches
            )
        );
        NamedCommands.registerCommand(
            "Tag8Right", 
            new GoToPosePID(
                drivetrain, 
                8, 
                VisionConstants.reefXOffsetInches, 
                VisionConstants.reefYRightOffsetInches
            )
        );

        // SCORE CORAL ON REEF
        NamedCommands.registerCommand(
            "ScoreL4", 
            new ElevateToReefLevelCmd(4, coralCollector, elevator)
                .withTimeout(2)
                .andThen(
                    Commands.parallel(
                        new ElevateToReefLevelCmd(4, coralCollector, elevator),
                        coralCollector.outtakeCommand()
                    ).withTimeout(0.75)
                )
                .andThen(
                new InstantCommand(
                    () -> {
                        System.out.println("L4 SCORE COMPLETE!");
                    }
                )
            )
        );

        // HUMAN PLAYER STATION
        final double goToHPTagTimeout = 2;

        NamedCommands.registerCommand(
            "IntakeCoral", 
            new ParallelCommandGroup(
                new ElevateToSetpointCmd(elevator, ElevatorConstants.humanPlayerHeight),
                coralCollector.setTiltSetpointCmd(CoralCollectorConstants.humanPlayerAngle),
                coralCollector.intakeCommand()    
            ).until(() -> coralCollector.inputs.hasCoral)
            .andThen(
                new InstantCommand(
                    () -> {
                        System.out.println("CORAL INTAKE COMPLETE!");
                    }
                )
            )
        );
        NamedCommands.registerCommand(
            "HPStation12", 
            new GoToPosePID(
                drivetrain,
                12,
                VisionConstants.hpXOffsetInches,
                VisionConstants.hpYOffsetInches
            ).withTimeout(goToHPTagTimeout)
        );
        NamedCommands.registerCommand(
            "HPStation13",
            new GoToPosePID(
                drivetrain, 
                13, 
                VisionConstants.hpXOffsetInches,
                VisionConstants.hpYOffsetInches
            ).withTimeout(goToHPTagTimeout)
        );
        NamedCommands.registerCommand(
            "HPStation2",
            new GoToPosePID(
                drivetrain, 
                2, 
                VisionConstants.hpXOffsetInches,
                VisionConstants.hpYOffsetInches
            ).withTimeout(goToReefTagTimeout)
        );
        NamedCommands.registerCommand(
            "HPStation1",
            new GoToPosePID(
                drivetrain, 
                1, 
                VisionConstants.hpXOffsetInches,
                VisionConstants.hpYOffsetInches
            ).withTimeout(goToHPTagTimeout)
        );

        // ELEVATOR RESET
        NamedCommands.registerCommand(
            "ResetElevator", 
            Commands.parallel(
                new ElevateToSetpointCmd(elevator, ElevatorConstants.levelHeights[0]),
                coralCollector.setTiltSetpointCmd(CoralCollectorConstants.levelCoralTiltAngle[0])
            ).until(() -> (elevator.inputs.zeroLimit || elevator.inputs.positionMeters <= 0.1))
            .andThen(
                new InstantCommand(
                    () -> {
                        System.out.println("ELEVATOR RESET COMPLETE!");
                    }
                )
            )
        );

        NamedCommands.registerCommand(
            "RightPathfindToClosestLowerTag",
            new PathfindToReefCmd(drivetrain, vision, true)
            .andThen(
                new InstantCommand(
                    () -> {
                        // Elastic.sendNotification(new Elastic.Notification(Elastic.Notification.NotificationLevel.INFO, "RIGHT REEF PATHFIND COMPLETE!", ));
                        System.out.println("RIGHT REEF PATHFIND COMPLETE!");
                    }
                )
            )
        );
        NamedCommands.registerCommand(
            "LeftPathfindToClosestLowerTag",
            new PathfindToReefCmd(drivetrain, vision, false)
            .andThen(
                new InstantCommand(
                    () -> {
                        System.out.println("LEFT REEF PATHFIND COMPLETE!");
                    }
                )
            )
        );

        NamedCommands.registerCommand(
            "PathfindToNearestHPS", 
            new PathfindToHPS(drivetrain, vision)
            .andThen(
                new InstantCommand(
                    () -> {
                        System.out.println("PATHFIND HUMAN PLAYER COMPLETE!");
                    }
                )
            )
        );
    }
}
