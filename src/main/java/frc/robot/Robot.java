// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.utils.LocalADStarAK;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    // DO THIS FIRST
    Pathfinding.setPathfinder(new LocalADStarAK());
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

    if (isReal()) {
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      new PowerDistribution(55, ModuleType.kRev); // Enables power distribution logging
    } else {
      setUseTiming(false); // Run as fast as possible
      String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in
    // the "Understanding Data Flow" page
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
                    // be added.

    // CameraServer.startAutomaticCapture();
    // webcam.setResolution(1280/10, 720/10);
    // SmartDashboard.putNumber("fps", webcam.getActualFPS());
    // SmartDashboard.putNumber("byterate", webcam.getActualDataRate());

    // m_robotContainer.getSwerveSubsytem().getNavX().reset();
    // m_robotContainer.getSwerveSubsytem().getOdometry().resetPosition(new
    // Pose2d(), new Rotation2d());
    // PathPlannerServer.startServer(5811); // 5811 = port number. adjust this
    // according to your needs

    // SmartDashboard.putData(CommandScheduler.getInstance());
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // if (RobotContainer.getShooter().isNotePresent()) {
    // RobotContainer.getShooter().indexerMotor.set(0);
    // RobotContainer.getCollector().rollerMotor.set(0);
    // }

    SmartDashboard.putData("cmdScheduler", CommandScheduler.getInstance());

    try {
      Optional<Alliance> ally = DriverStation.getAlliance();
      if (ally.isPresent()) {
        if (ally.get() == Alliance.Red) {
          RobotContainer.isRed = true;
        }
        if (ally.get() == Alliance.Blue) {
          RobotContainer.isRed = false;

        }
      } else {
        System.out.println("NO COLOR YET");
      }
    } catch (NullPointerException ex) {

      RobotContainer.isRed = RobotContainer.allianceChooser.getSelected();
    }
    SmartDashboard.putBoolean("isRED?", RobotContainer.isRed);

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    // m_robotContainer.getPneumatics().setExtenderState(Value.kReverse);
    // m_robotContainer.getExtendArm().setServoAngle(Constants.ArmConstants.restServoAngle);
    // m_robotContainer.getPneumatics().setGripperState(Value.kReverse);
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    // m_robotContainer.getExtendArm().setServoAngle(Constants.ArmConstants.restServoAngle);
    // m_robotContainer.getSwerveSubsytem().getNavX().reset();
    // m_robotContainer.getSwerveSubsytem().getOdometry().resetPosition(new
    // Rotation2d(),
    // m_robotContainer.getSwerveSubsytem().getModulePositions(), new Pose2d());
    try {
      m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    } catch (NullPointerException ex) {
      m_autonomousCommand = new InstantCommand();
    }

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    } else {
      DriverStation.reportError("m_autonomousCommand null in Robot.java", m_autonomousCommand == null);
      new InstantCommand().schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // m_robotContainer.getSwerveBase().resetOdometry(new Pose2d(0, 0, new
    // Rotation2d(Math.PI * 2)));
    // m_robotContainer.getSwerveBase().resetOdometry(new Pose2d(0, 0, new
    // Rotation2d(Units.degreesToRadians(0))));

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

}
