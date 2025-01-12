// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
  private final PhotonCamera camera; // Declare name of camera used in pipeline
  private List<PhotonPipelineResult> results; // stores all data the Photonvision returns
  private boolean hasTarget = false;
  private PhotonPipelineResult result; // stores  latest data that Photonvision returns

  /** Creates a new Vision. */
  public Vision() {
    // OV9782
    camera = new PhotonCamera(VisionConstants.cameraName);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    results = camera.getAllUnreadResults(); // query all unread results from PhotonVision
    if (!results.isEmpty()) {
      result = results.get(results.size() - 1); // get latest result from list of all results
      hasTarget = result.hasTargets(); // if camera has detected an AprilTag, 
    }
    Logger.recordOutput("HAS TARGET?", hasTarget);
  }
}
