package org.team2059.Wonko.subsystems.vision;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public interface VisionIO {
    @AutoLog
    class VisionIOInputs {

        public boolean lowerIsConnected = false;

        public double timestamp;
        
        public boolean hasLowerTarget = false;

        public int lowerBestTargetID = -1;

        public PhotonTrackedTarget lowerBestTarget = null;

        public boolean lowerIsOn = true;
    }

    default public void updateInputs(VisionIOInputs inputs) {};

    default public Optional<EstimatedRobotPose> getEstimatedLowerGlobalPose() {
        return Optional.empty();
    };
    default public Matrix<N3, N1> getLowerCurrentStdDevs() {
        return null;
    }
}
