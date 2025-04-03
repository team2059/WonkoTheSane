package org.team2059.Wonko.commands.vision;

import java.util.Set;

import org.team2059.Wonko.Constants.VisionConstants;
import org.team2059.Wonko.subsystems.drive.Drivetrain;
import org.team2059.Wonko.subsystems.vision.Vision;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * Command which goes to either the left or right stem of a reef tag using pathfinding.
 * The best tag on feed must be a reef tag for this command to run.
 * 
 * Pathfinds to 40in distance from center of robot to tag,
 * then aligns to either left or right side of reef using PID in x, y, theta dimensions.
 */

public class PathfindToReefAutoCmd extends SequentialCommandGroup{
    
    Drivetrain drivetrain;
    Vision vision;

    Transform3d tagToGoal;

    boolean isRight;

    public PathfindToReefAutoCmd (
        Drivetrain drivetrain,
        Vision vision,
        boolean isRight
    ){
        this.drivetrain = drivetrain; 
        this.vision = vision;

        this.isRight = isRight;

        // This is our ideal end state: 40in back and centered on tag (relative to robot center)

        if (isRight) {
            tagToGoal = new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(VisionConstants.reefXOffsetInches),
                    Units.inchesToMeters(VisionConstants.reefYRightOffsetInches),
                    0
                ),
                new Rotation3d(0, 0, Math.PI)
            );
        } else {
            tagToGoal = new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(VisionConstants.reefXOffsetInches),
                    Units.inchesToMeters(VisionConstants.reefYLeftOffsetInches),
                    0
                ),
                new Rotation3d(0, 0, Math.PI)
            );
        }

        // Require both vision & drivetrain subsystems
        // This ensures no conflicting commands will be run
        addRequirements(vision, drivetrain);

        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
            new InstantCommand(() -> {vision.inputs.upperIsOn = false;}),
            new InstantCommand(() -> drivetrain.stopAllMotors()),
            new DeferredCommand(() -> getPathfindCommand(), Set.of(drivetrain, vision)),
            new InstantCommand(() -> drivetrain.stopAllMotors()),
            new InstantCommand(() -> {vision.inputs.upperIsOn = true;})
        );
    }
    
    /** 
     * Generate our pathfinding command, which is run in sequence before PID alignment.
     * 
     * If no tag is detected, the tag is too ambigious, or the tag is not a valid reef tag, 
     * we return an InstantCommand (does nothing).
     * 
     * @return the resulting command
     */
    public Command getPathfindCommand() {
        if (
            vision.inputs.hasLowerTarget &&
            vision.inputs.lowerBestTarget != null &&
            vision.inputs.lowerBestTarget.getPoseAmbiguity() <= 0.2 &&
            VisionConstants.redReefTags.contains(vision.inputs.lowerBestTargetID) || VisionConstants.blueReefTags.contains(vision.inputs.lowerBestTargetID)
        ) {
            
            // Grab pose of tag
            int tagId = vision.inputs.lowerBestTargetID;
            var targetPose = VisionConstants.aprilTagFieldLayout.getTagPose(tagId);

            // Calculate end state
            var goalPose = targetPose.get().transformBy(tagToGoal).toPose2d();

            return AutoBuilder.pathfindToPose(
                goalPose, 
                new PathConstraints(
                    2.5, 
                    1.5,
                    Units.degreesToRadians(540),
                    Units.degreesToRadians(720)
                )
            );

        } else {
            return new InstantCommand(
                () -> {
                    System.out.println("Conditions Not Met For Auto Alignment To Reef");
                }
            );
        }
    }
}
