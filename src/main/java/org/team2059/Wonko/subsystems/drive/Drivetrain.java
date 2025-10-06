// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2059.Wonko.subsystems.drive;

import com.ctre.phoenix6.hardware.Pigeon2;
import gg.questnav.questnav.PoseFrame;
import org.littletonrobotics.junction.Logger;
import org.team2059.Wonko.Constants;
import org.team2059.Wonko.Constants.AutoConstants;
import org.team2059.Wonko.Constants.DrivetrainConstants;
import org.team2059.Wonko.Constants.VisionConstants;
import org.team2059.Wonko.routines.DrivetrainRoutine;
import org.team2059.Wonko.subsystems.oculus.Oculus;
import org.team2059.Wonko.subsystems.vision.Vision;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.team2059.Wonko.Constants.OculusConstants.ROBOT_TO_QUEST;

public class Drivetrain extends SubsystemBase {

  public static boolean fieldRelativeStatus = true;

  public final SwerveModule frontLeft;
  public final SwerveModule frontRight;
  public final SwerveModule backLeft;
  public final SwerveModule backRight;

  public final Oculus oculus;

  private final SwerveDrivePoseEstimator poseEstimator;

  public final DrivetrainRoutine routine;

  private final Field2d field = new Field2d();

  private final Pigeon2 gyro = new Pigeon2(50);

  public Drivetrain(Vision vision, Oculus oculus) {

    /*
     * Construct four SwerveModules
     *
     * Arguments: ID, then SwerveModuleIO:
     * - Drive motor can ID
     * - Rotation motor can ID
     * - Cancoder can ID
     * - Cancoder offset in radians
     * - Boolean drive inverter
     * - Boolean rotation inverter
     *
     * 1 frontLeft
     * 2 frontRight
     * 3 backLeft
     * 4 backRight
     */

    frontLeft = new SwerveModule(
      1,
      new MK4nVortexModule(
        DrivetrainConstants.frontLeftDriveMotorId,
        DrivetrainConstants.frontLeftRotationMotorId,
        DrivetrainConstants.frontLeftCanCoderId,
        DrivetrainConstants.frontLeftOffsetRad,
        false,
        true
      ));
    frontRight = new SwerveModule(
      2,
      new MK4nVortexModule(
        DrivetrainConstants.frontRightDriveMotorId,
        DrivetrainConstants.frontRightRotationMotorId,
        DrivetrainConstants.frontRightCanCoderId,
        DrivetrainConstants.frontRightOffsetRad,
        false,
        true
      ));
    backLeft = new SwerveModule(
      3,
      new MK4nVortexModule(
        DrivetrainConstants.backLeftDriveMotorId,
        DrivetrainConstants.backLeftRotationMotorId,
        DrivetrainConstants.backLeftCanCoderId,
        DrivetrainConstants.backLeftOffsetRad,
        false,
        true
      ));
    backRight = new SwerveModule(
      4,
      new MK4nVortexModule(
        DrivetrainConstants.backRightDriveMotorId,
        DrivetrainConstants.backRightRotationMotorId,
        DrivetrainConstants.backRightCanCoderId,
        DrivetrainConstants.backRightOffsetRad,
        false,
        true
      ));

    this.oculus = oculus;

    // initialize CANcoder offsets
    frontLeft.io.initRotationOffset();
    frontRight.io.initRotationOffset();
    backLeft.io.initRotationOffset();
    backRight.io.initRotationOffset();

    // reset encoders upon each start
    frontLeft.io.resetEncoders();
    frontRight.io.resetEncoders();
    backLeft.io.resetEncoders();
    backRight.io.resetEncoders();

    // SysID routine
    routine = new DrivetrainRoutine(this);

    // Estimates our pose on the field using vision, if available.
    // Behaves just like SwerveDriveOdometry, just with optional vision
    // measurements.
    poseEstimator = new SwerveDrivePoseEstimator(
      DrivetrainConstants.kinematics,
      getHeading(),
      getModulePositions(),
      new Pose2d(),
      VisionConstants.stateStdDevs,
      VisionConstants.measurementStdDevs);

    // Configure auto builder last
    configureAutoBuilder();

    PathPlannerLogging.setLogTargetPoseCallback((pose) -> { // target pose
      field.getObject("target pose").setPose(pose);
    });
    PathPlannerLogging.setLogActivePathCallback((poses) -> { // active path (list of poses)
      field.getObject("trajectory").setPoses(poses);
    });

    SmartDashboard.putData(field);
  }

  /**
   * @return Current robot pose in meters
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Reset odometry to a certain pose,
   * uses current module positions and heading
   *
   * @param pose specified Pose2d
   */
  public void resetOdometry(Pose2d pose) {
    poseEstimator.resetPosition(getHeading(), getModulePositions(), pose);
  }

  /**
   * @return ChassisSpeeds of current robot-relative speeds
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return DrivetrainConstants.kinematics.toChassisSpeeds(getStates());
  }

  /**
   * Set heading to zero
   */
  public void zeroHeading() {
    gyro.reset();
  }

  /**
   * @return Rotation2d of current navX heading
   */
  public Rotation2d getHeading() {
    return gyro.getRotation2d();
  }

  public void setPose(Pose2d targetPose) {
    oculus.setRobotPose(targetPose);
  }

  /**
   * @return current swerve module positions in SwerveModulePosition[] array
   */
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[]{
      new SwerveModulePosition(frontLeft.inputs.drivePosition,
        new Rotation2d(frontLeft.inputs.azimuthAbsolutePosition)),
      new SwerveModulePosition(frontRight.inputs.drivePosition,
        new Rotation2d(frontRight.inputs.azimuthAbsolutePosition)),
      new SwerveModulePosition(backLeft.inputs.drivePosition,
        new Rotation2d(backLeft.inputs.azimuthAbsolutePosition)),
      new SwerveModulePosition(backRight.inputs.drivePosition,
        new Rotation2d(backRight.inputs.azimuthAbsolutePosition))
    };
  }

  /**
   * Method to drive robot-relative
   *
   * @param chassisSpeeds desired speeds
   */
  public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);

    SwerveModuleState[] newStates = Constants.DrivetrainConstants.kinematics.toSwerveModuleStates(discreteSpeeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(newStates, Constants.DrivetrainConstants.maxVelocity);

    setModuleStates(newStates);
  }

  /**
   * Method to drive field-relative
   *
   * @param chassisSpeeds desired speeds
   */
  public void driveFieldRelative(ChassisSpeeds chassisSpeeds) {
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);

    chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(discreteSpeeds, getHeading());

    SwerveModuleState[] newStates = Constants.DrivetrainConstants.kinematics.toSwerveModuleStates(chassisSpeeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(newStates, Constants.DrivetrainConstants.maxVelocity);

    setModuleStates(newStates);
  }

  /**
   * @return current swerve module states of all modules
   */
  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];

    states[0] = frontLeft.io.getState();
    states[1] = frontRight.io.getState();
    states[2] = backLeft.io.getState();
    states[3] = backRight.io.getState();

    return states;
  }

  /**
   * Method to set module states
   *
   * @param desiredStates SwerveModuleState[] desired states
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    // makes it never go above specified max velocity
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DrivetrainConstants.maxVelocity);

    Logger.recordOutput("Desired States", desiredStates);

    // Sets the speed and rotation of each module
    frontLeft.io.setState(desiredStates[0], false);
    frontRight.io.setState(desiredStates[1], false);
    backLeft.io.setState(desiredStates[2], false);
    backRight.io.setState(desiredStates[3], false);
  }

  /**
   * Method to drive the robot either field or robot relative
   *
   * @param forward forward/backward linear velocity component
   * @param strafe strafe linear velocity component
   * @param rotation rotational component
   * @param isFieldRelative should drive field relative or not
   */
  public void drive(double forward, double strafe, double rotation, boolean isFieldRelative) {

    /*
     * ChassisSpeeds object to represent the overall state of the robot
     * ChassisSpeeds takes a forward and sideways linear value and a rotational
     * value
     *
     * speeds is set to field relative or default (robot relative) based on
     * parameter
     */

    ChassisSpeeds speeds = isFieldRelative
      ? ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, rotation, getHeading())
      : new ChassisSpeeds(forward, strafe, rotation);

    speeds = ChassisSpeeds.discretize(speeds, 0.02);

    // use kinematics (wheel placements) to convert overall robot state to array of
    // individual module states
    SwerveModuleState[] states = DrivetrainConstants.kinematics.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(states, DrivetrainConstants.maxVelocity);

    setModuleStates(states);

  }

  /**
   * Switch fieldRelativeStatus boolean to the opposite value
   */
  public void setFieldRelativity() {
    if (fieldRelativeStatus) {
      fieldRelativeStatus = false;
    } else {
      fieldRelativeStatus = true;
    }
  }

  /**
   * Method to configure AutoBuilder (make sure to do this last)
   */
  public void configureAutoBuilder() {

    System.out.println("Configuring Auto Builder...");

    try {
      RobotConfig config = RobotConfig.fromGUISettings();

      // Configure AutoBuilder
      AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier, MUST be robot relative
        (speeds) -> driveRobotRelative(speeds), // Method that will drive the robot given robot-relative chassis speeds
        new PPHolonomicDriveController(
          new PIDConstants(AutoConstants.kAutoTranslationP, 0.0, AutoConstants.kAutoTranslationD),
          new PIDConstants(AutoConstants.kAutoRotationP, 0.0, AutoConstants.kAutoRotationD)),
        config,
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field
          // The origin will remain on the blue side
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // reference to this subsystem to set requirements
      );
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  // Stop all motors in every swerve module
  public void stopAllMotors() {
    frontLeft.io.stop();
    frontRight.io.stop();
    backLeft.io.stop();
    backRight.io.stop();
  }

  // For drivetrain translation routine. Must lock wheels, so instead we use PID
  public void setModulesToZeroRadPID() {
    frontLeft.io.setAzimuthAngle(0);
    frontRight.io.setAzimuthAngle(0);
    backLeft.io.setAzimuthAngle(0);
    backRight.io.setAzimuthAngle(0);
  }

  @Override
  public void periodic() {

    // For safety...
    if (DriverStation.isDisabled()) {
      stopAllMotors();
    }

    if (oculus.isTracking()) {
      // Get the latest pose data frames
      PoseFrame[] questFrames = oculus.getPoseFrames();

      // Loop over pose data frames and send to pose estimator
      for (PoseFrame f : questFrames) {
        // Get quest pose
        Pose2d questPose = f.questPose();

        // Get timestamp for when data was sent
        poseEstimator.addVisionMeasurement(
          f.questPose().transformBy(ROBOT_TO_QUEST.inverse()),
          f.dataTimestamp(),
          Constants.OculusConstants.stdDevs
        );
      }
    }

    // Update pose estimator as if it were simply Odometry
    poseEstimator.update(getHeading(), getModulePositions());

    // Logging
    Logger.recordOutput("Pose", getPose());
    field.setRobotPose(getPose());
    Logger.recordOutput("Field-Relative?", fieldRelativeStatus);
    Logger.recordOutput("Real States", getStates());

    Logger.recordOutput("PigeonYaw", getHeading());
  }
}
