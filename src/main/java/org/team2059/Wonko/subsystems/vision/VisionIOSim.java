package org.team2059.Wonko.subsystems.vision;

import java.util.List;
import java.util.Optional;
import org.team2059.Wonko.RobotContainer;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.team2059.Wonko.Constants.VisionConstants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;

public class VisionIOSim implements VisionIO {

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

    // For simulation
    private VisionSystemSim visionSim;
    private PhotonCameraSim upperCamSim;
    private PhotonCameraSim lowerCamSim;
    
    public VisionIOSim() {
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

        visionSim = new VisionSystemSim("main");
        visionSim.addAprilTags(aprilTagFieldLayout);

        SimCameraProperties cameraProperties = new SimCameraProperties();
        cameraProperties.setCalibration(1280, 720, Rotation2d.fromDegrees(78));
        // Approximate detection noise with average and standard deviation error in pixels.
        cameraProperties.setCalibError(0.38, 0.2);
        // Set the camera image capture framerate (Note: this is limited by robot loop rate).
        cameraProperties.setFPS(30);
        // The average and standard deviation in milliseconds of image data latency.
        cameraProperties.setAvgLatencyMs(35);
        cameraProperties.setLatencyStdDevMs(5);

        upperCamSim = new PhotonCameraSim(upperCamera, cameraProperties);
        lowerCamSim = new PhotonCameraSim(lowerCamera, cameraProperties);

        visionSim.addCamera(upperCamSim, VisionConstants.upperCameraToRobot);
        visionSim.addCamera(lowerCamSim, VisionConstants.lowerCameraToRobot);

        upperCamSim.enableProcessedStream(true);
        lowerCamSim.enableProcessedStream(true);
        upperCamSim.enableDrawWireframe(true);
        lowerCamSim.enableDrawWireframe(true);

    }

    // Update vision inputs with latest target info from each camera
    @Override
    public void updateInputs(VisionIOInputs inputs) {

        visionSim.update(RobotContainer.drivetrain.getPose());

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
