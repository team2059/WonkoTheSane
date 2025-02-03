// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package org.team2059.Wonko.commands;

// import org.photonvision.targeting.PhotonTrackedTarget;
// import org.team2059.Wonko.subsystems.Drivetrain;
// import org.team2059.Wonko.subsystems.Vision;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj2.command.Command;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class TurnParallelToReef extends Command {

//   private final Drivetrain drivetrain;
//   private final Vision vision;
//   private double rotationSpeed;
//   private final int tagID;
//   private double lastKnownZ = 0.0;

//   /** Creates a new TurnParallelToTag. */
//   public TurnParallelToReef(Drivetrain drivetrain, Vision vision, int tagID) {
//     this.drivetrain = drivetrain;
//     this.vision = vision;
//     this.tagID = tagID;

//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(drivetrain, vision);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     vision.turnController.setTolerance(0.01);
//     vision.turnController.enableContinuousInput(-Math.PI, Math.PI);
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {

//     PhotonTrackedTarget t = vision.getCertainLowerTarget(tagID);

//     if (vision.lowerHasTargets && t != null && t.getPoseAmbiguity() <= 0.2 || lastKnownZ != 0.0) {

//       lastKnownZ = t.getBestCameraToTarget().getRotation().getZ();

//       rotationSpeed = -MathUtil.clamp(vision.turnController.calculate(lastKnownZ, Math.PI), -1, 1);

//       drivetrain.drive(0, 0, rotationSpeed, true);

//     } else {
//       this.cancel();
//     }
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     rotationSpeed = 0;
//     drivetrain.drive(0,0,0,true);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return vision.turnController.atSetpoint();
//   }
// }
