package org.team2059.Wonko.commands.vision;

import java.util.List;
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

public class PathfindtoClosestTag extends SequentialCommandGroup {
    Drivetrain drivetrain;
    Vision vision;
    PhotonTrackedTarget targetToUse;
    Transform3d tagToGoal;
    boolean usingUpperCamera;
    private int idTagToChase;

    public PathfindtoClosestTag(
            Drivetrain drivetrain,
            Vision vision,
            double frontOffsetInches,
            double yOffsetInches) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.tagToGoal = new Transform3d(
                new Translation3d(Units.inchesToMeters(frontOffsetInches), Units.inchesToMeters(yOffsetInches), 0),
                new Rotation3d(0, 0, Math.PI));

        addRequirements(vision, drivetrain);

        addCommands(
                new DeferredCommand(() -> getCommand(), Set.of(drivetrain, vision)),
                new InstantCommand(() -> drivetrain.stopAllMotors()));
    }

    private PhotonTrackedTarget getBestTarget(List<PhotonTrackedTarget> targets) {
        if (targets.isEmpty())
            return null;

        PhotonTrackedTarget bestTarget = targets.get(0);
        for (PhotonTrackedTarget target : targets) {
            if (target.getPoseAmbiguity() < bestTarget.getPoseAmbiguity()) {
                bestTarget = target;
            }
        }
        return bestTarget;
    }

    public Command getCommand() {
        if (!vision.hasTargets()) {
            return new InstantCommand();
        }

        // Compare ambiguity between cameras and select the better one
        if (vision.inputs.hasUpperTarget && vision.inputs.hasLowerTarget) {
            // if both see tags then use the one with less ambiguity
            if (vision.inputs.upperBestTarget.getPoseAmbiguity() < 
                vision.inputs.lowerBestTarget.getPoseAmbiguity()) {
                targetToUse = vision.inputs.upperBestTarget;
                usingUpperCamera = true;
            } else {
                targetToUse = vision.inputs.lowerBestTarget;
                usingUpperCamera = false;
            }
        } else if (vision.inputs.hasUpperTarget) {
            // Only upper camera sees a tag
            targetToUse = vision.inputs.upperBestTarget;
            usingUpperCamera = true;
        } else if (vision.inputs.hasLowerTarget) {
            // Only lower camera sees a tag
            targetToUse = vision.inputs.lowerBestTarget;
            usingUpperCamera = false;
        } else {
            return new InstantCommand();
        }

        var robotPose2d = drivetrain.getPose();
        var robotPose3d = new Pose3d(
                robotPose2d.getX(),
                robotPose2d.getY(),
                0,
                new Rotation3d(0, 0, robotPose2d.getRotation().getRadians()));

        // Get the transformation from the camera to the tag
        var camToTarget = targetToUse.getBestCameraToTarget();

        // Transform the robot's pose to to find the tag's pose(upper or lower camera)
        var cameraPose = robotPose3d.transformBy(
                usingUpperCamera ? VisionConstants.upperCameraToRobot : VisionConstants.lowerCameraToRobot);
        var targetPose = cameraPose.transformBy(camToTarget);

        // Transform the tag's pose to set the goal
        var goalPose = targetPose.transformBy(tagToGoal).toPose2d();

        Logger.recordOutput("goalPose", goalPose);
        Logger.recordOutput("selectedTagId", targetToUse.getFiducialId());
        Logger.recordOutput("usingUpperCamera", usingUpperCamera);
        Logger.recordOutput("tagAmbiguity", targetToUse.getPoseAmbiguity());

        return AutoBuilder.pathfindToPose(
            goalPose,
            new PathConstraints(
                2.0,
                1.5,
                Units.degreesToRadians(540),
                Units.degreesToRadians(720)));
    }
}