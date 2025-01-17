// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;

public class Drivetrain extends SubsystemBase {

  public static boolean fieldRelativeStatus = true;

  // Create 4 SwerveModule objects using given constants.
  private final SwerveModule frontLeft = new SwerveModule(
    DrivetrainConstants.frontLeftDriveMotorId, 
    DrivetrainConstants.frontLeftRotationMotorId, 
    DrivetrainConstants.frontLeftCanCoderId, 
    DrivetrainConstants.frontLeftOffsetRad);
  private final SwerveModule frontRight = new SwerveModule(
    DrivetrainConstants.frontRightDriveMotorId, 
    DrivetrainConstants.frontRightRotationMotorId, 
    DrivetrainConstants.frontRightCanCoderId, 
    DrivetrainConstants.frontRightOffsetRad);
  private final SwerveModule backLeft = new SwerveModule(
    DrivetrainConstants.backLeftDriveMotorId, 
    DrivetrainConstants.backLeftRotationMotorId, 
    DrivetrainConstants.backLeftCanCoderId, 
    DrivetrainConstants.backLeftOffsetRad);
  private final SwerveModule backRight = new SwerveModule(
    DrivetrainConstants.backRightDriveMotorId, 
    DrivetrainConstants.backRightRotationMotorId, 
    DrivetrainConstants.backRightCanCoderId, 
    DrivetrainConstants.backRightOffsetRad);

  // Create NavX object (gyro)
  private final AHRS navX;

  // Create swerve drive odometry engine, used to track robot on field
  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(Constants.DrivetrainConstants.kinematics, new Rotation2d(), getModulePositions());

  public Drivetrain() {

    // NavX may need an extra second to start...
    navX = new AHRS(AHRS.NavXComType.kMXP_SPI);
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        navX.reset();
        odometry.resetPosition(new Rotation2d(), getModulePositions(), new Pose2d());
      } catch (Exception e) {
      }
    }).start();

    // initialize CANcoder offsets
    frontLeft.initRotationOffset();
    frontRight.initRotationOffset();
    backLeft.initRotationOffset();
    backRight.initRotationOffset();

    // drive motor inversions: offsets mess with these sometimes
    frontLeft.setMotorInversion(frontLeft.getDriveMotor(), false);
    frontRight.setMotorInversion(frontRight.getDriveMotor(), true);
    backLeft.setMotorInversion(backLeft.getDriveMotor(), false);
    backRight.setMotorInversion(backRight.getDriveMotor(), true);

    // rotation motor inversions: all or nothing situation
    frontLeft.setMotorInversion(frontLeft.getRotationMotor(), true);
    frontRight.setMotorInversion(frontRight.getRotationMotor(), true);
    backLeft.setMotorInversion(backLeft.getRotationMotor(), true);
    backRight.setMotorInversion(backRight.getRotationMotor(), true);

    // reset encoders upon each start
    frontLeft.resetEncoders();
    frontRight.resetEncoders();
    backLeft.resetEncoders();
    backRight.resetEncoders();

    // Configure auto builder last
    configureAutoBuilder();
  }

  /**
   * @return Current robot pose in meters
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
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
    odometry.resetPosition(getHeading(), getModulePositions(), pose);
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
      new SwerveModulePosition(frontLeft.getCurrentDistanceMeters(), frontLeft.getCANcoderRad()),
      new SwerveModulePosition(frontRight.getCurrentDistanceMeters(), frontRight.getCANcoderRad()),
      new SwerveModulePosition(backLeft.getCurrentDistanceMeters(), backLeft.getCANcoderRad()),
      new SwerveModulePosition(backRight.getCurrentDistanceMeters(), backRight.getCANcoderRad())
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
    setModuleStates(newStates);
  }

  /**
   * Method to drive field-relative
   * @param chassisSpeeds
   */
  public void driveFieldRelative(ChassisSpeeds chassisSpeeds) {
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
    discreteSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(discreteSpeeds, getHeading());
    SwerveModuleState[] newStates = Constants.DrivetrainConstants.kinematics.toSwerveModuleStates(discreteSpeeds);
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
    // makes it never go above 5 m/s
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DrivetrainConstants.maxVelocity);
    // Sets the speed and rotation of each module
    frontLeft.setDesiredStateClosedLoop(desiredStates[0]);
    frontRight.setDesiredStateClosedLoop(desiredStates[1]);
    backLeft.setDesiredStateClosedLoop(desiredStates[2]);
    backRight.setDesiredStateClosedLoop(desiredStates[3]);

    Logger.recordOutput("Target States", desiredStates);
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
    // Fetch RobotConfig from GUI settings
    try {
      // Configure AutoBuilder
      AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier, MUST be robot relative 
        (speeds) -> driveRobotRelative(speeds), // Method that will drive the robot given robot-relative chassisspeeds 
        new PPHolonomicDriveController(
          new PIDConstants(10, 0, 0), // Translation PID constants 
          new PIDConstants(0, 0, 0)), // Rotation PID constants
        new RobotConfig(
          DrivetrainConstants.kMass, 
          DrivetrainConstants.kMomentOfIntertia, 
          new ModuleConfig(
            DrivetrainConstants.wheelDiameter / 2, 
            DrivetrainConstants.maxVelocity, 
            DrivetrainConstants.kWheelCoF, 
            DCMotor.getNEO(1).withReduction(DrivetrainConstants.driveGearRatio), 
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(getHeading(), getModulePositions());

    Logger.recordOutput("NavX Angle (Degrees)", -navX.getAngle());
    
    Logger.recordOutput("Real States", getStates());
    Logger.recordOutput("Pose", getPose());

    Logger.recordOutput("FIELD-RELATIVE?", fieldRelativeStatus);
  }
}
