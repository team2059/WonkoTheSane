// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2059.Wonko.commands.vision;

import java.util.ArrayList;
import java.util.Set;

import org.team2059.Wonko.Constants.VisionConstants;
import org.team2059.Wonko.subsystems.drive.Drivetrain;
import org.team2059.Wonko.subsystems.vision.Vision;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PathfindReefOffset extends SequentialCommandGroup {
  
  // Dependent subsystems
  Drivetrain drivetrain;
  Vision vision;
  ArrayList<Integer> reefIDS;  

  private boolean right; 

  public PathfindReefOffset(
    Drivetrain drivetrain,
    Vision vision, 
    ArrayList<Integer> reefIDS,
    boolean right 
  ) {  
    
    this.drivetrain = drivetrain;
    this.vision = vision; 

    this.reefIDS = reefIDS; 
    this.right = right; 

    addRequirements(vision, drivetrain);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DeferredCommand(() -> getCommand(), Set.of(drivetrain, vision)),
      new InstantCommand(() -> drivetrain.stopAllMotors())
    );
  }
  
  public Command getCommand() {
    System.out.println(right);
    if (vision.inputs.hasLowerTarget && reefIDS.contains(vision.inputs.lowerBestTargetID)) {
      if (right) {
        return new PathfindToAnyTagCmd
            (drivetrain, 
            vision, 
            vision.inputs.lowerBestTargetID, 
            17.5, 
            VisionConstants.reefOffset); 
      } else { 
        return new PathfindToAnyTagCmd
            (drivetrain, 
            vision, 
            vision.inputs.lowerBestTargetID, 
            17.5, 
            -VisionConstants.reefOffset); 
      }
    }

    return new InstantCommand(); 
  }
  
  }
