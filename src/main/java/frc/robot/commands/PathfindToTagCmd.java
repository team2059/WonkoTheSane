// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.auto.AutoBuilder;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.SwerveBase;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PathfindToTagCmd extends SequentialCommandGroup {
        SwerveBase swerveBase;
        Vision vision;
        PhotonTrackedTarget targetToUse;

        Transform3d TAG_TO_GOAL;

        private int ID_OF_TAG_TO_CHASE;

        /** Creates a new SequentialChaseTagCmd. */
        public PathfindToTagCmd(SwerveBase swerveBase,
                        Vision vision, int ID_OF_TAG_TO_CHASE, double frontOffsetInches) {

                this.vision = vision;
                this.swerveBase = swerveBase;
                this.ID_OF_TAG_TO_CHASE = ID_OF_TAG_TO_CHASE;
                this.TAG_TO_GOAL = new Transform3d(new Translation3d(Units.inchesToMeters(frontOffsetInches), 0, 0),
                                new Rotation3d(0, 0, 0));

                addRequirements(vision, swerveBase);
                addCommands(new DeferredCommand(() -> getCommand(), Set.of(swerveBase, vision)),
                                new InstantCommand(() -> swerveBase.stopModules()));
        }

        public Command getCommand() {

                // if (RobotContainer.isRed) {
                // ID_OF_TAG_TO_CHASE = 4;
                // System.out.println("RED" + ID_OF_TAG_TO_CHASE);

                // } else {
                // ID_OF_TAG_TO_CHASE = 7;
                // System.out.println("BLUE" + ID_OF_TAG_TO_CHASE);
                // }

                var robotPose2d = swerveBase.getPose();

                var robotPose3d = new Pose3d(robotPose2d.getX(), robotPose2d.getY(), 0,
                                new Rotation3d(0, 0, robotPose2d.getRotation().getRadians()));

                var result = vision.getCamera().getLatestResult();

                if (result.hasTargets() == false) {
                        return new InstantCommand();
                } else {
                        try {
                                var allTargets = result.getTargets();
                                for (PhotonTrackedTarget target : allTargets) {

                                        if (target.getFiducialId() == ID_OF_TAG_TO_CHASE) {
                                                targetToUse = target;

                                        }
                                }

                                // this doesn't really do anything unless "always do single target estimation is
                                // checked -> by deafulat, returns -1"
                                if (targetToUse.getPoseAmbiguity() >= 0.2) {
                                        return new InstantCommand();
                                }

                                // System.out.println("ID: " + targetToUse.getFiducialId() + " ambig = "
                                // + targetToUse.getPoseAmbiguity());

                                // Get the transformation from the camera to the tag
                                var camToTarget = targetToUse.getBestCameraToTarget();

                                // Transform the robot's pose to find the tag's pose
                                var cameraPose = robotPose3d.transformBy(VisionConstants.robotToCam);
                                var targetPose = cameraPose.transformBy(camToTarget);

                                // Transform the tag's pose to set our goal
                                var goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();

                                System.out.println(goalPose.toString());

                                return AutoBuilder.pathfindToPose(goalPose, new PathConstraints(
                                                2, 1.5,
                                                Units.degreesToRadians(540), Units.degreesToRadians(720)), 0);

                        } catch (NullPointerException ex) {
                                ex.printStackTrace();
                                return new InstantCommand();
                        }

                }
        }

}