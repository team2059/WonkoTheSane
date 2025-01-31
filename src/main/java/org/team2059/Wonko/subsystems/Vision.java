// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2059.Wonko.subsystems;

import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.team2059.Wonko.Constants.VisionConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  // Declare name of camera used in pipeline
  private final PhotonCamera upperCamera;
  private final PhotonCamera lowerCamera;

  // Store all data that Photonvision returns
  private List<PhotonPipelineResult> upperCameraResults; 
  private List<PhotonPipelineResult> lowerCameraResults;

  public boolean hasAnyTargets = false;
  public boolean upperHasTargets = false;
  public boolean lowerHasTargets = false;

  // Store latest data that Photonvision returns
  private PhotonPipelineResult upperCameraResult;
  private PhotonPipelineResult lowerCameraResult;

  /** Creates a new Vision. */
  public Vision() {
    // Arducam OV9782
    upperCamera = new PhotonCamera(VisionConstants.upperCameraName);
    lowerCamera = new PhotonCamera(VisionConstants.lowerCameraName);
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
