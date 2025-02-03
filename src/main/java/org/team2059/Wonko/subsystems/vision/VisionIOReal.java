package org.team2059.Wonko.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.team2059.Wonko.Constants.VisionConstants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class VisionIOReal implements VisionIO {

    // Declare name of cameras uses in pipelines
    private final PhotonCamera upperCamera;
    private final PhotonCamera lowerCamera;

    // Store all data that PhotonVision returns
    private List<PhotonPipelineResult> upperCameraResults;
    private List<PhotonPipelineResult> lowerCameraResults;

    // Store latest data that PhotonVision returns
    private PhotonPipelineResult upperCameraResult;
    private PhotonPipelineResult lowerCameraResult;

    // AprilTag field layout to derive locations from
    private AprilTagFieldLayout aprilTagFieldLayout;

    // PhotonPoseEstimators, one for each pipeline
    private PhotonPoseEstimator upperPoseEstimator;
    private PhotonPoseEstimator lowerPoseEstimator;
    
    public VisionIOReal() {
        aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

        upperCamera = new PhotonCamera(VisionConstants.upperCameraName);
        lowerCamera = new PhotonCamera(VisionConstants.lowerCameraName);

        upperPoseEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
            VisionConstants.upperCameraToRobot
        );
        lowerPoseEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
            VisionConstants.lowerCameraToRobot
        );

        upperPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        lowerPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    // Update vision inputs with latest target info from each camera
    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // Update camera readings
        upperCameraResults = upperCamera.getAllUnreadResults();
        lowerCameraResults = lowerCamera.getAllUnreadResults();

        if (!upperCameraResults.isEmpty()) {
            // Get latest results from list of results
            upperCameraResult = upperCameraResults.get(upperCameraResults.size() - 1);

            // Set boolean value
            inputs.hasUpperTarget = upperCameraResult.hasTargets();
            if (inputs.hasUpperTarget) inputs.upperBestTargetID = upperCameraResult.getBestTarget().getFiducialId();
        }

        if (!lowerCameraResults.isEmpty()) {
            lowerCameraResult = lowerCameraResults.get(lowerCameraResults.size() - 1);

            inputs.hasLowerTarget = lowerCameraResult.hasTargets();
            if (inputs.hasLowerTarget) inputs.lowerBestTargetID = lowerCameraResult.getBestTarget().getFiducialId();
        }
    }

    @Override
    public Optional<EstimatedRobotPose> getEstimatedUpperGlobalPose() {
        return upperPoseEstimator.update(upperCameraResult);
    }

    @Override
    public Optional<EstimatedRobotPose> getEstimatedLowerGlobalPose() {
        return lowerPoseEstimator.update(lowerCameraResult);
    }
}
