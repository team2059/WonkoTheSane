package org.team2059.Wonko.commands;

import static edu.wpi.first.units.Units.Radians;

import org.team2059.Wonko.Constants.CoralCollectorConstants;
import org.team2059.Wonko.Constants.ElevatorConstants;
import org.team2059.Wonko.commands.elevator.ElevateToSetpointCmd;
import org.team2059.Wonko.commands.vision.AlignToHumanPlayer;
import org.team2059.Wonko.commands.vision.AlignToReef;
import org.team2059.Wonko.subsystems.algae.AlgaeCollector;
import org.team2059.Wonko.subsystems.coral.CoralCollector;
import org.team2059.Wonko.subsystems.drive.Drivetrain;
import org.team2059.Wonko.subsystems.elevator.Elevator;
import org.team2059.Wonko.subsystems.vision.Vision;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/** 
 * Organization class for registering all
 * NamedCommands to be referenced in auto.
 */
public final class AutoCommands {

    /**
     * Returns an InstantCommand which prints a message to the console for use in log replay.
     * 
     * @param message the desired message to display in the console
     * @return an InstantCommand
     */
    public static Command logToConsoleCommand(String message) {
        return new InstantCommand(() -> System.out.println(message));
    }

    /**
     * Register all NamedCommands for PathPlanner. Requires all used subsystems to be passed.
     * @param drivetrain
     * @param coralCollector
     * @param algaeCollector
     * @param elevator
     * @param vision
     */
    public static void registerNamedCommands(
        Drivetrain drivetrain,
        CoralCollector coralCollector,
        AlgaeCollector algaeCollector,
        Elevator elevator,
        Vision vision
    ) {

        /* Timeouts */
        final double alignToReefTimeout = 1.5;

        /* Coral Score */
        NamedCommands.registerCommand(
            "ScoreL4", 
            new ElevateToReefLevelCmd(4, coralCollector, elevator)
                .until(
                    // () -> Math.abs(elevator.inputs.positionMeters - ElevatorConstants.levelHeights[4].in(Meters)) <= 0.04
                    () -> Math.abs(coralCollector.inputs.tiltAbsPosRadians - CoralCollectorConstants.levelCoralTiltAngle[4].in(Radians)) <= 0.05
                )
                .andThen(
                    Commands.parallel(
                        new ElevateToReefLevelCmd(4, coralCollector, elevator),
                        coralCollector.outtakeCommand()
                    ).withTimeout(0.5)
                )
                .andThen(logToConsoleCommand("[auto] L4 SCORE COMPLETE!"))
        );

        /* Coral Intake */
        NamedCommands.registerCommand(
            "IntakeCoral", 
            coralCollector.autoIntakeCmd()
            .withTimeout(1.5)
            .andThen(logToConsoleCommand("[auto] CORAL INTAKE COMPLETE!"))
        );

        /* Elevator Reset */
        NamedCommands.registerCommand(
            "ResetElevator", 
            Commands.parallel(
                new ElevateToSetpointCmd(elevator, ElevatorConstants.levelHeights[0]),
                coralCollector.setTiltSetpointCmd(CoralCollectorConstants.levelCoralTiltAngle[0])
            ).until(() -> (elevator.inputs.zeroLimit || elevator.inputs.positionMeters <= 0.1))
            .andThen(logToConsoleCommand("[auto] ELEVATOR RESET COMPLETE!"))
        );

        /* Align to Reef */
        NamedCommands.registerCommand(
            "AlignToReefLeft", 
            new AlignToReef(drivetrain, vision, false, false)
            .withTimeout(alignToReefTimeout)
            .andThen(logToConsoleCommand("[auto] LEFT REEF ALIGN COMPLETE!"))
        );
        NamedCommands.registerCommand(
            "AlignToReefRight", 
            new AlignToReef(drivetrain, vision, false, false)
            .withTimeout(alignToReefTimeout)
            .andThen(logToConsoleCommand("[auto] RIGHT REEF ALIGN COMPLETE!"))
        );

        final double humanPlayerAlignTimeout = 1.5;
        /* Human Player Station */
        NamedCommands.registerCommand(
            "AlignToHumanPlayer", 
            Commands.sequence(
                new AlignToHumanPlayer(drivetrain, vision, false)
                .withTimeout(humanPlayerAlignTimeout),
                logToConsoleCommand("[auto] HUMAN PLAYER ALIGN COMPLETE!")
            )
        );
    }
}
