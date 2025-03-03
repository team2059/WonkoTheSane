package org.team2059.Wonko.subsystems.vision;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

public interface VisionIO {
    @AutoLog
    class VisionIOInputs {

        public boolean upperIsConnected = false;
        public boolean lowerIsConnected = false;

        public double timestamp;
        
        public boolean hasUpperTarget = false;
        public boolean hasLowerTarget = false;

        public int upperBestTargetID = -1;
        public int lowerBestTargetID = -1;

        public PhotonTrackedTarget upperBestTarget = null;
        public PhotonTrackedTarget lowerBestTarget = null;
    }

    default public void updateInputs(VisionIOInputs inputs) {};

    default public Optional<EstimatedRobotPose> getEstimatedUpperGlobalPose() {
        return Optional.empty();
    };
    default public Optional<EstimatedRobotPose> getEstimatedLowerGlobalPose() {
        return Optional.empty();
    };
}
