// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2059.Wonko.subsystems;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.team2059.Wonko.Constants.VisionConstants;
import org.team2059.Wonko.util.LoggedTunableNumber;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  // Declare name of camera used in pipeline
  public final PhotonCamera upperCamera;
  public final PhotonCamera lowerCamera;

  // Store all data that Photonvision returns
  private List<PhotonPipelineResult> upperCameraResults; 
  private List<PhotonPipelineResult> lowerCameraResults;

  // Store latest data that Photonvision returns
  private PhotonPipelineResult upperCameraResult;
  private PhotonPipelineResult lowerCameraResult;

  public boolean hasAnyTargets = false;
  public boolean upperHasTargets = false;
  public boolean lowerHasTargets = false;

  private AprilTagFieldLayout aprilTagFieldLayout;
  private PhotonPoseEstimator upperPoseEstimator;
  private PhotonPoseEstimator lowerPoseEstimator;

  // COMMAND STUFF
  private final LoggedTunableNumber turnP = new LoggedTunableNumber("Vision/TurnP", 1);
  private final LoggedTunableNumber turnD = new LoggedTunableNumber("Vision/TurnD", 0.02);
  public PIDController turnController = new PIDController(turnP.get(), 0, turnD.get());

  public static Vision instance;
  public static Vision getInstance() {
    if (instance == null) {
      instance = new Vision();
    }
    return instance;
  }

  /** Creates a new Vision. */
  public Vision() {
    // Arducam OV9782
    upperCamera = new PhotonCamera(VisionConstants.upperCameraName);
    lowerCamera = new PhotonCamera(VisionConstants.lowerCameraName);

    try {
      aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);
    } catch (IOException e) {
      e.printStackTrace();
    }

    upperPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.upperCameraToRobot);
    lowerPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.lowerCameraToRobot);

    upperPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    lowerPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Update camera readings
    upperCameraResults = upperCamera.getAllUnreadResults();
    lowerCameraResults = lowerCamera.getAllUnreadResults();
    if (!upperCameraResults.isEmpty()) {
      // Get latest result from list of results
      upperCameraResult = upperCameraResults.get(upperCameraResults.size() - 1);

      // Set boolean value
      upperHasTargets = upperCameraResult.hasTargets();
    }
    if (!lowerCameraResults.isEmpty()) {
      // Get latest result from list of results
      lowerCameraResult = lowerCameraResults.get(lowerCameraResults.size() - 1);

      // Set boolean value
      lowerHasTargets = lowerCameraResult.hasTargets();
    }

    // Update any targets bool
    if (lowerHasTargets || upperHasTargets) {
      hasAnyTargets = true;
    } else {
      hasAnyTargets = false;
    }

    Logger.recordOutput("Any Targets", hasAnyTargets);
    Logger.recordOutput("Lower Targets", lowerHasTargets);
    Logger.recordOutput("Upper Targets", upperHasTargets);

    // Update tunables
    if (turnP.hasChanged(hashCode()) || turnD.hasChanged(hashCode())) {
      turnController.setP(turnP.get());
      turnController.setD(turnD.get());
    }
  }

  public Optional<EstimatedRobotPose> getUpperEstimatedGlobalPose() {
    return upperPoseEstimator.update(upperCameraResult);
  }

  public Optional<EstimatedRobotPose> getLowerEstimatedRobotPose() {
    return lowerPoseEstimator.update(lowerCameraResult);
  }

  public PhotonTrackedTarget getCertainUpperTarget(int id) {
    if (upperCameraResult.hasTargets()) {
      for (var target : upperCameraResult.getTargets()) {
        if (target.getFiducialId() == id) {
          return target;
        }
      }
    }

    return null;
  }

  public PhotonTrackedTarget getCertainLowerTarget(int id) {
    if (lowerCameraResult.hasTargets()) {
      for (var target : lowerCameraResult.getTargets()) {
        if (target.getFiducialId() == id) {
          return target;
        }
      }
    }

    return null;
  }

  public PhotonCamera getUpperCamera() {
    return upperCamera;
  }

  public PhotonCamera getLowerCamera() {
    return lowerCamera;
  }
}
