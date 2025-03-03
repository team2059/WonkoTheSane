package org.team2059.Wonko.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    public boolean hasTargets() {
        return inputs.hasLowerTarget || inputs.hasUpperTarget;
    }
}
