// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2059.Wonko.subsystems;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.team2059.Wonko.Constants;
import org.team2059.Wonko.Constants.AutoConstants;
import org.team2059.Wonko.Constants.DrivetrainConstants;
import org.team2059.Wonko.Constants.VisionConstants;
import org.team2059.Wonko.routines.DrivetrainRoutine;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

  public static boolean fieldRelativeStatus = true;

  // Create 4 SwerveModule objects using given constants.
  public final SwerveModule frontLeft = new SwerveModule(
    DrivetrainConstants.frontLeftDriveMotorId, 
    DrivetrainConstants.frontLeftRotationMotorId, 
    DrivetrainConstants.frontLeftCanCoderId, 
    DrivetrainConstants.frontLeftOffsetRad,
    false,
    true);
  public final SwerveModule frontRight = new SwerveModule(
    DrivetrainConstants.frontRightDriveMotorId, 
    DrivetrainConstants.frontRightRotationMotorId, 
    DrivetrainConstants.frontRightCanCoderId, 
    DrivetrainConstants.frontRightOffsetRad,
    true,
    true);
  public final SwerveModule backLeft = new SwerveModule(
    DrivetrainConstants.backLeftDriveMotorId, 
    DrivetrainConstants.backLeftRotationMotorId, 
    DrivetrainConstants.backLeftCanCoderId, 
    DrivetrainConstants.backLeftOffsetRad,
    false,
    true);
  public final SwerveModule backRight = new SwerveModule(
    DrivetrainConstants.backRightDriveMotorId, 
    DrivetrainConstants.backRightRotationMotorId, 
    DrivetrainConstants.backRightCanCoderId, 
    DrivetrainConstants.backRightOffsetRad,
    false,
    true);

  // Create NavX object (gyro)
  private final AHRS navX;

  // Create swerve drive odometry engine, used to track robot on field
  // private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(Constants.DrivetrainConstants.kinematics, new Rotation2d(), getModulePositions());

  private SwerveDrivePoseEstimator poseEstimator;

  private final Vision vision;

  public final DrivetrainRoutine drivetrainRoutine;

  public Drivetrain(Vision vision) {

    this.vision = vision;

    // NavX may need an extra second to start...
    navX = new AHRS(AHRS.NavXComType.kMXP_SPI);
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        navX.reset();
      } catch (Exception e) {
        e.printStackTrace();
      }
    }).start();

    // initialize CANcoder offsets
    frontLeft.initRotationOffset();
    frontRight.initRotationOffset();
    backLeft.initRotationOffset();
    backRight.initRotationOffset();

    // reset encoders upon each start
    frontLeft.resetEncoders();
    frontRight.resetEncoders();
    backLeft.resetEncoders();
    backRight.resetEncoders();

    // Configure auto builder last
    configureAutoBuilder();

    SmartDashboard.putData("Swerve Drive", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");

        builder.addDoubleProperty("Front Left Angle", () -> frontLeft.getCANcoderRad().getRadians(), null);
        builder.addDoubleProperty("Front Left Velocity", () -> frontLeft.getDriveVelocity(), null);

        builder.addDoubleProperty("Front Right Angle", () -> frontRight.getCANcoderRad().getRadians(), null);
        builder.addDoubleProperty("Front Right Velocity", () -> frontRight.getDriveVelocity(), null);

        builder.addDoubleProperty("Back Left Angle", () -> backLeft.getCANcoderRad().getRadians(), null);
        builder.addDoubleProperty("Back Left Velocity", () -> backLeft.getDriveVelocity(), null);

        builder.addDoubleProperty("Back Right Angle", () -> backRight.getCANcoderRad().getRadians(), null);
        builder.addDoubleProperty("Back Right Velocity", () -> backRight.getDriveVelocity(), null);

        builder.addDoubleProperty("Robot Angle", () -> getHeading().getRadians(), null);
      }
    });

    drivetrainRoutine = new DrivetrainRoutine(this);

    poseEstimator = new SwerveDrivePoseEstimator(
      DrivetrainConstants.kinematics, 
      getHeading(), 
      getModulePositions(), 
      new Pose2d(), 
      VisionConstants.stateStdDevs,
      VisionConstants.measurementStdDevs);

  }

  /**
   * @return Current robot pose in meters
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * @return AHRS navX object
   */
  public AHRS getNavX() {
    return navX;
  }

  /**
   * Reset odometry to a certain pose,
   * uses current module positions and heading
   * 
   * @param pose specified Pose2d
   */
  public void resetOdometry(Pose2d pose) {
    Logger.recordOutput("Commanded Pose", pose);
    poseEstimator.resetPosition(getHeading(), getModulePositions(), pose);
  }

  /**
   * @return ChassisSpeeds of current robot-relative speeds
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    ChassisSpeeds chassisSpeeds = DrivetrainConstants.kinematics.toChassisSpeeds(getStates());

    return chassisSpeeds;
  }

  /**
   * Set navX heading to zero
   */
  public void zeroHeading() {
    navX.reset();
  }

  /**
   * @return Rotation2d of current navX heading
   */
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-navX.getYaw());
  }

  /**
   * @return current swerve module positions in SwerveModulePosition[] array
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = {
      new SwerveModulePosition(frontLeft.getDriveEncoderPosition(), frontLeft.getCANcoderRad()),
      new SwerveModulePosition(frontRight.getDriveEncoderPosition(), frontRight.getCANcoderRad()),
      new SwerveModulePosition(backLeft.getDriveEncoderPosition(), backLeft.getCANcoderRad()),
      new SwerveModulePosition(backRight.getDriveEncoderPosition(), backRight.getCANcoderRad())
    };

    return positions;
  }

  /**
   * Method to drive robot-relative
   * @param chassisSpeeds
   */
  public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
    SwerveModuleState[] newStates = Constants.DrivetrainConstants.kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(newStates, Constants.DrivetrainConstants.maxVelocity);
    
    SmartDashboard.putData("Commanded Swerve States", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");

        builder.addDoubleProperty("Front Left Angle", () -> newStates[0].angle.getRadians(), null);
        builder.addDoubleProperty("Front Left Velocity", () -> newStates[0].speedMetersPerSecond, null);

        builder.addDoubleProperty("Front Right Angle", () -> newStates[1].angle.getRadians(), null);
        builder.addDoubleProperty("Front Right Velocity", () -> newStates[1].speedMetersPerSecond, null);

        builder.addDoubleProperty("Back Left Angle", () -> newStates[2].angle.getRadians(), null);
        builder.addDoubleProperty("Back Left Velocity", () -> newStates[2].speedMetersPerSecond, null);

        builder.addDoubleProperty("Back Right Angle", () -> newStates[3].angle.getRadians(), null);
        builder.addDoubleProperty("Back Right Velocity", () -> newStates[3].speedMetersPerSecond, null);

        builder.addDoubleProperty("Robot Angle", () -> getHeading().getRadians(), null);
      }
    });

    setModuleStates(newStates);
  }

  /**
   * Method to drive field-relative
   * @param chassisSpeeds
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
    states[0] = frontLeft.getState();
    states[1] = frontRight.getState();
    states[2] = backLeft.getState();
    states[3] = backRight.getState();

    return states;
  }

  /**
   * Method to set module states
   * @param desiredStates SwerveModuleState[] desired states
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    Logger.recordOutput("Desired States", desiredStates);
    // makes it never go above specified max velocity
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DrivetrainConstants.maxVelocity);
    // Sets the speed and rotation of each module
    frontLeft.setState(desiredStates[0], false);
    frontRight.setState(desiredStates[1], false);
    backLeft.setState(desiredStates[2], false);
    backRight.setState(desiredStates[3], false);
  }
  
  /**
   * Method to drive the robot either field or robot relative
   * @param forward
   * @param strafe
   * @param rotation
   * @param isFieldRelative
   */
  public void drive(double forward, double strafe, double rotation, boolean isFieldRelative) {

    /**
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
      // Configure AutoBuilder
      AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier, MUST be robot relative 
        (speeds) -> driveRobotRelative(speeds), // Method that will drive the robot given robot-relative chassisspeeds 
        new PPHolonomicDriveController(
          new PIDConstants(AutoConstants.kAutoTranslationP, 0.0, AutoConstants.kAutoTranslationD),
          new PIDConstants(AutoConstants.kAutoRotationP, 0.0, AutoConstants.kAutoRotationD)
        ),
        new RobotConfig(
          DrivetrainConstants.kMass, 
          DrivetrainConstants.kMomentOfIntertia, 
          new ModuleConfig(
            DrivetrainConstants.wheelDiameter / 2, 
            DrivetrainConstants.maxVelocity, 
            DrivetrainConstants.kWheelCoF, 
            DCMotor.getNeoVortex(1).withReduction(DrivetrainConstants.driveGearRatio), 
            DrivetrainConstants.driveCurrentLimit, 
            1
          ), 
          DrivetrainConstants.trackWidth
        ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
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

  public void stopAllMotors() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    // odometry.update(getHeading(), getModulePositions());    

    final Optional<EstimatedRobotPose> upperOptional = vision.getUpperEstimatedGlobalPose();
    final Optional<EstimatedRobotPose> lowerOptional = vision.getLowerEstimatedRobotPose();

    if (upperOptional.isPresent()) {
      poseEstimator.addVisionMeasurement(
        upperOptional.get().estimatedPose.toPose2d(),
        upperOptional.get().timestampSeconds
      );
    }
    if (lowerOptional.isPresent()) {
      poseEstimator.addVisionMeasurement(
        lowerOptional.get().estimatedPose.toPose2d(), 
        lowerOptional.get().timestampSeconds
      );
    }

    poseEstimator.update(getHeading(), getModulePositions());

    Logger.recordOutput("Pose", getPose());
    Logger.recordOutput("Field-Relative?", fieldRelativeStatus);
  }
}

