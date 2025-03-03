package org.team2059.Wonko.commands.vision;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.team2059.Wonko.subsystems.drive.Drivetrain;
import org.team2059.Wonko.subsystems.vision.Vision;
import org.team2059.Wonko.util.LoggedTunableNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class TurnParallelToReef extends Command {

    private final Drivetrain drivetrain;
    private final Vision vision;

    private PIDController turnController;

    private double rotationSpeed;

    private int tagID;

    private LoggedTunableNumber kP = new LoggedTunableNumber("Vision/AlignReef/kP", 4.0);
    private LoggedTunableNumber kI = new LoggedTunableNumber("Vision/AlignReef/kI", 0.0);
    private LoggedTunableNumber kD = new LoggedTunableNumber("Vision/AlignReef/kD", 0.0);

    private PhotonTrackedTarget t;
    private double z;

    public TurnParallelToReef(Drivetrain drivetrain, Vision vision, int tagID) {
        this.drivetrain = drivetrain;
        this.vision = vision;

        this.tagID = tagID;

        turnController = new PIDController(kP.get(), kI.get(), kD.get());

        rotationSpeed = 0;
        z = 0;

        addRequirements(drivetrain, vision);
    }
    
    @Override
    public void initialize() {
        turnController.setTolerance(0.01);
        turnController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void execute() {

        LoggedTunableNumber.ifChanged(
            hashCode(), 
            () -> {
                turnController.setPID(kP.get(), kI.get(), kD.get());
            }, 
            kP, kI, kD
        );

        t = vision.inputs.lowerBestTarget;

        if (t != null && t.getFiducialId() == tagID) {
            z = t.getBestCameraToTarget().getRotation().getZ();

            rotationSpeed = -MathUtil.clamp(
                turnController.calculate(z, Math.PI),
                -1, 
                1
            );

            Logger.recordOutput("z", z);
            Logger.recordOutput("rotationSpeed", rotationSpeed);

            drivetrain.drive(0, 0, rotationSpeed, true);
        } else {
            this.cancel();
        }
    }

    @Override
    public void end(boolean interrupted) {
        rotationSpeed = 0;
        z = 0;
        t = null;
        drivetrain.drive(0, 0, 0, true);
    }

    @Override
    public boolean isFinished() {
        return turnController.atSetpoint();
    }
}