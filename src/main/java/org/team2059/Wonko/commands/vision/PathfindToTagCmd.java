// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2059.Wonko.commands.vision;

import java.util.Set;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.team2059.Wonko.Constants.VisionConstants;
import org.team2059.Wonko.subsystems.drive.Drivetrain;
import org.team2059.Wonko.subsystems.vision.Vision;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PathfindToTagCmd extends SequentialCommandGroup {

  Drivetrain drivetrain;
  Vision vision;
  PhotonTrackedTarget targetToUse;

  Transform3d tagToGoal;

  private int idTagToChase;

  /** Creates a new PathfindToTagCmd. */
  public PathfindToTagCmd(
    Drivetrain drivetrain,
    Vision vision, 
    int idTagToChase, 
    double frontOffsetInches
  ) {

    this.drivetrain = drivetrain;
    this.vision = vision;
    this.idTagToChase = idTagToChase;
    this.tagToGoal = new Transform3d(
      new Translation3d(Units.inchesToMeters(frontOffsetInches), 0, 0),
      new Rotation3d(0, 0, 0)
    );

    addRequirements(vision, drivetrain);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DeferredCommand(() -> getCommand(), Set.of(drivetrain, vision)),
      new InstantCommand(() -> drivetrain.stopAllMotors())
    );
  }

  public Command getCommand() {
    var robotPose2d = drivetrain.getPose();

    var robotPose3d = new Pose3d(
      robotPose2d.getX(),
      robotPose2d.getY(),
      0,
      new Rotation3d(0, 0, robotPose2d.getRotation().getRadians())
    );

    if (vision.inputs.lowerBestTargetID == idTagToChase) {
      targetToUse = vision.inputs.lowerBestTarget;

      // Get the transformation from the camera to the tag
      var camToTarget = targetToUse.getBestCameraToTarget();

      // Transform the robot's pose to to find the tag's pose
      var cameraPose = robotPose3d.transformBy(VisionConstants.lowerCameraToRobot);
      var targetPose = cameraPose.transformBy(camToTarget);

      // Transform the tag's pose to set the goal
      var goalPose = targetPose.transformBy(tagToGoal).toPose2d();

      Logger.recordOutput("goalPose", goalPose);

      return AutoBuilder.pathfindToPose(
        goalPose,
        new PathConstraints(
          2, 
          1.5, 
          Units.degreesToRadians(540), 
          Units.degreesToRadians(720)
        )
      );
    } else {
      return new InstantCommand();
    }
    // if (!vision.inputs.hasLowerTarget) {
    //   return new InstantCommand();
    // } else {
    //   if (vision.inputs.lowerBestTargetID == idTagToChase) {

    //   }
    // }
  }
}
