package org.team2059.Wonko.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.team2059.Wonko.Constants.VisionConstants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionIOReal implements VisionIO {

    // Declare name of cameras uses in pipelines
    private final PhotonCamera lowerCamera;

    // Store all data that PhotonVision returns
    private List<PhotonPipelineResult> lowerCameraResults;

    // Store latest data that PhotonVision returns
    private PhotonPipelineResult lowerCameraResult;

    // AprilTag field layout to derive locations from
    private AprilTagFieldLayout aprilTagFieldLayout;

    // PhotonPoseEstimators, one for each pipeline
    private PhotonPoseEstimator lowerPoseEstimator;

    private Matrix<N3, N1> lowerCurStdDevs;
    
    public VisionIOReal() {

        // AndyMark or welded field layout? Check at competition.
        aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

        lowerCamera = new PhotonCamera(VisionConstants.lowerCameraName);

        lowerPoseEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
            VisionConstants.lowerCameraToRobot
        );

        lowerPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    // Update vision inputs with latest target info from each camera
    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // Update connectivity status
        inputs.lowerIsConnected = lowerCamera.isConnected();
        lowerCameraResults = lowerCamera.getAllUnreadResults();

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

    private void updateLowerEstimationStdDevs(
        Optional<EstimatedRobotPose> estimatedPose,
        List<PhotonTrackedTarget> targets
    ) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            lowerCurStdDevs = VisionConstants.singleTagStdDevs;
        } else {
            // Pose present. Start running heuristic.
            var estStdDevs = VisionConstants.singleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we cound, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = lowerPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                    tagPose
                        .get()
                        .toPose2d()
                        .getTranslation()
                        .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                lowerCurStdDevs = VisionConstants.singleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Increase std devs if multiple targets are visible.
                if (numTags > 1) estStdDevs = VisionConstants.multiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4) {
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                } else {
                    estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                }
                lowerCurStdDevs = estStdDevs;
            }
        }
    }

    @Override
    public Optional<EstimatedRobotPose> getEstimatedLowerGlobalPose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var change : lowerCameraResults) {
            visionEst = lowerPoseEstimator.update(change);
            updateLowerEstimationStdDevs(visionEst, change.getTargets());
        }
        return visionEst;
    }

    @Override
    public Matrix<N3, N1> getLowerCurrentStdDevs() {
        return lowerCurStdDevs;
    }
}
