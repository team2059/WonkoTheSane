// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
  // Declare name of camera used in pipeline
  private final PhotonCamera camera;

  // Store all data that Photonvision returns
  private List<PhotonPipelineResult> results; 

  public boolean hasTargets = false;

  // Store latest data that Photonvision returns
  private PhotonPipelineResult result;

  /** Creates a new Vision. */
  public Vision() {
    // Arducam OV9782
    camera = new PhotonCamera(VisionConstants.cameraName);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Query all unread results from PhotonVision
    results = camera.getAllUnreadResults();
    if (!results.isEmpty()) {
      // Get latest result from list of all results
      result = results.get(results.size() - 1);

      // Set boolean value
      hasTargets = result.hasTargets();
    }

    Logger.recordOutput("HAS TARGETS?", hasTargets);
  }

  public PhotonTrackedTarget getCertainTarget(int id) {
    if (result.hasTargets()) {
      for (var target : result.getTargets()) {
        if (target.getFiducialId() == id) {
          return target;
        }
      }
    }

    return null;
  }

  public PhotonCamera getCamera() {
    return camera;
  }
}
