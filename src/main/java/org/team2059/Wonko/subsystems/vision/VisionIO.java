package org.team2059.Wonko.subsystems.vision;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;

public interface VisionIO {
    @AutoLog
    class VisionIOInputs {
        public double timestamp;
        
        public boolean hasUpperTarget = false;
        public boolean hasLowerTarget = false;

        public double upperLatencyMs = 0.0;
        public double lowerLatencyMs = 0.0;

        public int upperBestTargetID = -1;
        public int lowerBestTargetID = -1;
    }

    public void updateInputs(VisionIOInputs inputs);

    public Optional<EstimatedRobotPose> getEstimatedUpperGlobalPose();
    public Optional<EstimatedRobotPose> getEstimatedLowerGlobalPose();
}
