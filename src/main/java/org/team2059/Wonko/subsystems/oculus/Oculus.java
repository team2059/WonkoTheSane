package org.team2059.Wonko.subsystems.oculus;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import org.littletonrobotics.junction.Logger;

import static org.team2059.Wonko.Constants.OculusConstants.ROBOT_TO_QUEST;

public class Oculus extends SubsystemBase {
  private final QuestNav questNav = new QuestNav();

  private PoseFrame[] poseFrames;

  public Oculus() {
    poseFrames = questNav.getAllUnreadPoseFrames();
  }

  public void setRobotPose(Pose2d desiredPose) {
    questNav.setPose(desiredPose.transformBy(ROBOT_TO_QUEST));
  }

  public PoseFrame[] getPoseFrames() {
    return poseFrames;
  }

  public Pose2d getQuestPose() {
    if (poseFrames.length > 0) {
      return poseFrames[poseFrames.length - 1].questPose();
    } else {
      return null;
    }
  }

  public Pose2d getRobotPose() {
    if (poseFrames.length > 0) {
      return poseFrames[poseFrames.length - 1].questPose().transformBy(ROBOT_TO_QUEST.inverse());
    } else {
      return null;
    }
  }

  public boolean isTracking() {
    return questNav.isTracking();
  }

  @Override
  public void periodic() {
    questNav.commandPeriodic();

    poseFrames = questNav.getAllUnreadPoseFrames();

    Logger.recordOutput("QuestNavConnected", questNav.isConnected());
    Logger.recordOutput("QuestNavBatt", questNav.getBatteryPercent().getAsInt());
    Logger.recordOutput("QuestNavRobotPose", getRobotPose());
  }
}