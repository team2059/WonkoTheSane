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

        // AndyMark or welded field layout? Check at competition.
        aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

        upperCamera = new PhotonCamera(VisionConstants.upperCameraName);
        lowerCamera = new PhotonCamera(VisionConstants.lowerCameraName);

        upperPoseEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout, 
            PoseStrategy.LOWEST_AMBIGUITY, 
            VisionConstants.upperCameraToRobot
        );
        lowerPoseEstimator = new PhotonPoseEstimator(   
            aprilTagFieldLayout, 
            PoseStrategy.LOWEST_AMBIGUITY, 
            VisionConstants.lowerCameraToRobot
        );

        upperPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        lowerPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    // Update vision inputs with latest target info from each camera
    @Override
    public void updateInputs(VisionIOInputs inputs) {

        // Update connectivity status
        inputs.upperIsConnected = upperCamera.isConnected();
        inputs.lowerIsConnected = lowerCamera.isConnected();

        // Update camera readings
        upperCameraResults = upperCamera.getAllUnreadResults();
        lowerCameraResults = lowerCamera.getAllUnreadResults();

        if (!upperCameraResults.isEmpty()) {
            // Get latest results from list of results
            upperCameraResult = upperCameraResults.get(upperCameraResults.size() - 1);

            inputs.hasUpperTarget = upperCameraResult.hasTargets();
            if (inputs.hasUpperTarget) { // If we have any targets, get the best one.
                inputs.upperBestTarget = upperCameraResult.getBestTarget();
                inputs.upperBestTargetID = inputs.upperBestTarget.getFiducialId();
            } else { // If we don't have any targets, set null values.
                inputs.upperBestTargetID = -1;
                inputs.upperBestTarget = null;
            }
        }

        if (!lowerCameraResults.isEmpty()) {
            lowerCameraResult = lowerCameraResults.get(lowerCameraResults.size() - 1);

            inputs.hasLowerTarget = lowerCameraResult.hasTargets();
            if (inputs.hasLowerTarget) {
                inputs.lowerBestTarget = lowerCameraResult.getBestTarget();
                inputs.lowerBestTargetID = inputs.lowerBestTarget.getFiducialId();
            } else {
                inputs.lowerBestTargetID = -1;
                inputs.lowerBestTarget = null;
            }
        }
    }

    @Override
    public Optional<EstimatedRobotPose> getEstimatedUpperGlobalPose() {
        if (upperCameraResult != null) {
            return upperPoseEstimator.update(upperCameraResult);
        } else {
            return Optional.empty();
        }
    }

    @Override
    public Optional<EstimatedRobotPose> getEstimatedLowerGlobalPose() {
        if (lowerCameraResult != null) {
            return lowerPoseEstimator.update(lowerCameraResult);
        } else {
            return Optional.empty();
        }
    }
}
