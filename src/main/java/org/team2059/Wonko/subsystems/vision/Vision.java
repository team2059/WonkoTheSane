package org.team2059.Wonko.subsystems.vision;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    
    private final VisionIO io;
    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

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

    public Optional<EstimatedRobotPose> getEstimatedUpperGlobalPose() {
        return io.getEstimatedUpperGlobalPose();
    }

    public Optional<EstimatedRobotPose> getEstimatedLowerGlobalPose() {
        return io.getEstimatedLowerGlobalPose();
    }

    public boolean hasTargets() {
        return inputs.hasLowerTarget || inputs.hasUpperTarget;
    }
}
