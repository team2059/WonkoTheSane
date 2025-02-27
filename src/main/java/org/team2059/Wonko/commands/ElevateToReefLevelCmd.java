// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2059.Wonko.commands;

import static edu.wpi.first.units.Units.*;

import org.team2059.Wonko.Constants.CoralCollectorConstants;
import org.team2059.Wonko.Constants.ElevatorConstants;
import org.team2059.Wonko.commands.coral.TiltCoralToSetpointCmd;
import org.team2059.Wonko.commands.elevator.ElevateToSetpointCmd;
import org.team2059.Wonko.subsystems.coral.CoralCollector;
import org.team2059.Wonko.subsystems.elevator.Elevator;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ElevateToReefLevelCmd extends ParallelCommandGroup {

  /** Creates a new ElevateToReefLevelCmd. */
  public ElevateToReefLevelCmd(int targetLevel, CoralCollector coralCollector, Elevator elevator) {    

    /* 
     * Create a new parallel command group, which in parallel, will:
     * - Hold coral collector at specified resting position
     *   -> until elevator is within 0.1m of level, at which point hold coral collector at level tilt
     * - Elevate to the level's height.
     */
    addCommands(
      // Hold coral tilt at resting position 
      //  until elevator is within 0.1m of setpoint,
      //  then hold coral tilt at the outtake level
      new TiltCoralToSetpointCmd(coralCollector, CoralCollectorConstants.restingCoralCollectorPos)
        .until(() -> elevator.inputs.positionMeters >= ElevatorConstants.levelHeights[targetLevel].in(Meters) - 0.1)
        .andThen(new TiltCoralToSetpointCmd(coralCollector, CoralCollectorConstants.levelCoralTiltAngle[targetLevel])),
      // Elevate to the target level
      new ElevateToSetpointCmd(elevator, ElevatorConstants.levelHeights[targetLevel])
    );
  }
}