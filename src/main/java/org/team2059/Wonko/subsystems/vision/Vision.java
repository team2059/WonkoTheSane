package org.team2059.Wonko.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team2059.Wonko.RobotContainer;

public class Vision extends SubsystemBase {
    
    public final VisionIO io;
    public final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

    public Vision(VisionIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        // Update inputs
        io.updateInputs(inputs);

        // Process inputs
        Logger.processInputs("Vision", inputs);
    }

    // Sets Oculus to AprilTag pose
    public boolean syncWithOculus() {
        var lowerOptional = io.getEstimatedLowerGlobalPose();
        if (lowerOptional.isPresent()) {
            System.out.println("Syncing PhotonVision with Oculus");
            RobotContainer.oculus.setRobotPose(lowerOptional.get().estimatedPose.toPose2d());
            return true;
        } else {
            System.out.println("No tag present");
            return false;
        }
    }

    public boolean hasTargets() {
        return inputs.hasLowerTarget;
    }
}
