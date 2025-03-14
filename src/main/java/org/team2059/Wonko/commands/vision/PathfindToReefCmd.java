package org.team2059.Wonko.commands.vision;

import java.util.Set;

import org.littletonrobotics.junction.Logger;
import org.team2059.Wonko.Constants.VisionConstants;
import org.team2059.Wonko.subsystems.drive.Drivetrain;
import org.team2059.Wonko.subsystems.vision.Vision;
import org.team2059.Wonko.util.LoggedTunableNumber;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
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

public class PathfindToReefCmd extends SequentialCommandGroup{
    
    Drivetrain drivetrain;
    Vision vision;

    Transform3d tagToGoal;

    boolean isRight;

    public PathfindToReefCmd (
        Drivetrain drivetrain,
        Vision vision,
        boolean isRight
    ){
        this.drivetrain = drivetrain; 
        this.vision = vision;

        this.isRight = isRight;

        // This is our ideal end state: 40in back and centered on tag (relative to robot center)
        tagToGoal = new Transform3d(
            new Translation3d(
                Units.inchesToMeters(40),
                0,
                0
            ),
            new Rotation3d(0, 0, Math.PI)
        );

        // Require both vision & drivetrain subsystems
        // This ensures no conflicting commands will be run
        addRequirements(vision, drivetrain);

        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
            new DeferredCommand(() -> getPathfindCommand(), Set.of(drivetrain, vision)),
            new InstantCommand(() -> drivetrain.stopAllMotors())
        );
    }

    /** Command which allows us to "manually" drive using PID to do precise alignment. */
    public class AlignWithPID extends Command {
        double xTarget;
        double yTarget;
        double rotTarget;

        PIDController xController;
        PIDController yController;
        PIDController thetaController;

        LoggedTunableNumber pX = new LoggedTunableNumber("VisionAlign/pX", 2);
        LoggedTunableNumber pY = new LoggedTunableNumber("VisionAlign/pY", 3);
        LoggedTunableNumber pT = new LoggedTunableNumber("VisionAlign/pT", 2.5);

        /** Creates a new AlignWithPID command. */
        public AlignWithPID(
            double xTarget,
            double yTarget,
            double rotTarget
        ) {
            this.xTarget = xTarget;
            this.yTarget = yTarget;
            this.rotTarget = rotTarget;

            xController = new PIDController(pX.get(), 0, 0);
            yController = new PIDController(pY.get(), 0, 0);
            thetaController = new PIDController(pT.get(), 0, 0);

            addRequirements(drivetrain);
        }

        @Override
        public void initialize() {
            var goalPose = new Pose2d(
                xTarget, yTarget, Rotation2d.fromRadians(rotTarget)
            );

            if (isRight) { // RIGHT SIDE SHIFT
                goalPose = goalPose.transformBy(new Transform2d(Units.inchesToMeters(15), Units.inchesToMeters(6), Rotation2d.fromRadians(Math.PI)));
            } else { // LEFT SIDE SHIFT
                goalPose = goalPose.transformBy(new Transform2d(Units.inchesToMeters(15), Units.inchesToMeters(-9), Rotation2d.fromRadians(Math.PI)));
            }

            Logger.recordOutput("GOAL POSE", goalPose);

            xController.setSetpoint(goalPose.getX());
            yController.setSetpoint(goalPose.getY());
            thetaController.setSetpoint(goalPose.getRotation().getRadians());
            
            thetaController.enableContinuousInput(-Math.PI, Math.PI);

            xController.setTolerance(Units.inchesToMeters(1));
            yController.setTolerance(Units.inchesToMeters(1));
            thetaController.setTolerance(Units.degreesToRadians(5));
        }

        @Override
        public void execute() {
            LoggedTunableNumber.ifChanged(
                hashCode(), 
                () -> {
                    xController.setP(pX.get());
                    yController.setP(pY.get());
                    thetaController.setP(pT.get());
                }, 
                pX, pY, pT
            );

                var drivetrainPose = drivetrain.getPose();

            double xSpeed = xController.calculate(drivetrainPose.getX());
            double ySpeed = yController.calculate(drivetrainPose.getY());
            double thetaSpeed = thetaController.calculate(drivetrainPose.getRotation().getRadians());

            Logger.recordOutput("xSpeed", xSpeed);
            Logger.recordOutput("ySpeed", ySpeed);
            Logger.recordOutput("thetaSpeed", thetaSpeed);

            drivetrain.drive(
                MathUtil.clamp(-xSpeed, -1, 1), 
                MathUtil.clamp(-ySpeed, -1, 1), 
                MathUtil.clamp(thetaSpeed, -5, 5), 
                true
            );
        }

        @Override
        public void end(boolean interrupted) {
            drivetrain.drive(0, 0, 0, true);
        }

        @Override
        public boolean isFinished() {
            return (xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint());
        }
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
            VisionConstants.reefTags.contains(vision.inputs.lowerBestTargetID)
        ) {
            
            // Grab pose of tag
            int tagId = vision.inputs.lowerBestTargetID;
            var targetPose = VisionConstants.aprilTagFieldLayout.getTagPose(tagId);

            // Calculate end state
            var goalPose = targetPose.get().transformBy(tagToGoal).toPose2d();

            return AutoBuilder.pathfindToPose(
                goalPose, 
                new PathConstraints(
                    2, 
                    1.5,
                    Units.degreesToRadians(540),
                    Units.degreesToRadians(720)
                )
            ).andThen(
                new AlignWithPID(
                    VisionConstants.aprilTagFieldLayout.getTagPose(tagId).get().getX(), 
                    VisionConstants.aprilTagFieldLayout.getTagPose(tagId).get().getY(), 
                    VisionConstants.aprilTagFieldLayout.getTagPose(tagId).get().getRotation().getZ()
                )
            );

        } else {
            return new InstantCommand();
        }
    }
}
