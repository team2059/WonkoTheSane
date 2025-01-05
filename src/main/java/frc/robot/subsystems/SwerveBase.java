// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

public class SwerveBase extends SubsystemBase {

  public static boolean fieldRelativeStatus = true;

  // Create 4 SwerveModule objects using given constants.
  private final SwerveModule frontLeft = new SwerveModule(
    SwerveConstants.frontLeftDriveMotorId, 
    SwerveConstants.frontLeftRotationMotorId, 
    SwerveConstants.frontLeftCanCoderId, 
    SwerveConstants.frontLeftOffsetRad);
  private final SwerveModule frontRight = new SwerveModule(
    SwerveConstants.frontRightDriveMotorId, 
    SwerveConstants.frontRightRotationMotorId, 
    SwerveConstants.frontRightCanCoderId, 
    SwerveConstants.frontRightOffsetRad);
  private final SwerveModule backLeft = new SwerveModule(
    SwerveConstants.backLeftDriveMotorId, 
    SwerveConstants.backLeftRotationMotorId, 
    SwerveConstants.backLeftCanCoderId, 
    SwerveConstants.backLeftOffsetRad);
  private final SwerveModule backRight = new SwerveModule(
    SwerveConstants.backRightDriveMotorId, 
    SwerveConstants.backRightRotationMotorId, 
    SwerveConstants.backRightCanCoderId, 
    SwerveConstants.backRightOffsetRad);

  // Create NavX object (gyro)
  private final AHRS navX;

  // Create swerve drive odometry engine, used to track robot on field
  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(Constants.SwerveConstants.kinematics, new Rotation2d(), getModulePositions());

  public SwerveBase() {

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
    frontLeft.setMotorInversion(frontLeft.getDriveMotor(), true);
    frontRight.setMotorInversion(frontRight.getDriveMotor(), false);
    backLeft.setMotorInversion(backLeft.getDriveMotor(), false);
    backRight.setMotorInversion(backRight.getDriveMotor(), false);

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

    // configureAutoBuilder();
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public AHRS getNavX() {
    return navX;
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getHeading(), getModulePositions(), pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    ChassisSpeeds chassisSpeeds = SwerveConstants.kinematics.toChassisSpeeds(getStates());

    return chassisSpeeds;
  }

  public void zeroHeading() {
    navX.reset();
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-navX.getYaw());
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = {
      new SwerveModulePosition(-frontLeft.getCurrentDistanceMetersPerSecond(), frontLeft.getRotationEncoderPosition()),
      new SwerveModulePosition(-frontRight.getCurrentDistanceMetersPerSecond(), frontRight.getRotationEncoderPosition()),
      new SwerveModulePosition(-backLeft.getCurrentDistanceMetersPerSecond(), backLeft.getRotationEncoderPosition()),
      new SwerveModulePosition(-backRight.getCurrentDistanceMetersPerSecond(), backRight.getRotationEncoderPosition())
    };

    return positions;
  }

  public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
    SwerveModuleState[] newStates = Constants.SwerveConstants.kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(newStates, Constants.SwerveConstants.maxVelocity);
    setModuleStates(newStates);
  }

  public void driveFieldRelative(ChassisSpeeds chassisSpeeds) {
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
    discreteSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(discreteSpeeds, getHeading());
    SwerveModuleState[] newStates = Constants.SwerveConstants.kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(newStates, Constants.SwerveConstants.maxVelocity);
    setModuleStates(newStates);
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    states[0] = frontLeft.getState();
    states[1] = frontRight.getState();
    states[2] = backLeft.getState();
    states[3] = backRight.getState();

    return states;
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    // makes it never go above 5 m/s
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.maxVelocity);
    // Sets the speed and rotation of each module
    frontLeft.setDesiredStateClosedLoop(desiredStates[0]);
    frontRight.setDesiredStateClosedLoop(desiredStates[1]);
    backLeft.setDesiredStateClosedLoop(desiredStates[2]);
    backRight.setDesiredStateClosedLoop(desiredStates[3]);

    Logger.recordOutput("Target States", desiredStates);
  }
  
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
    SwerveModuleState[] states = SwerveConstants.kinematics.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.maxVelocity);

    setModuleStates(states);

  }

  public void setFieldRelativity() {
    if (fieldRelativeStatus) {
      fieldRelativeStatus = false;
    } else {
      fieldRelativeStatus = true;
    }
  }

  // public void configureAutoBuilder() {
  //   AutoBuilder.configureHolonomic(
  //     this::getPose, // robot pose supplier
  //     this::resetOdometry, // method to reset odometry (will be called if auto has a starting pose)
  //     this::getRobotRelativeSpeeds, // ChassisSpeeds supplier (must be robot relative)
  //     this::driveRobotRelative, // method that will drive the robot given robot relative ChassisSpeeds
  //     new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
  //       new PIDConstants(0, 0, 0), // Translation PID constants
  //       new PIDConstants(0, 0, 0), // Rotation PID constants
  //       SwerveConstants.maxVelocity, // Max module speed in m/s
  //       SwerveConstants.driveBaseRadius, // drive base radius in meters, distance from robot center to furthest module
  //       new ReplanningConfig()// Default path replanning config, see the API for options
  //     ),
  //     () -> {
  //       // Boolean supplier that controls when the path will be mirrored for the red alliance
  //       // This will flip the path being followed to the red side of the field.
  //       // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

  //       var alliance = DriverStation.getAlliance();
  //       if (alliance.isPresent()) {
  //         return alliance.get() == DriverStation.Alliance.Red;
  //       }
  //       return false;
  //     },
  //     this
  //   );
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(getHeading(), getModulePositions());

    SmartDashboard.putNumber("NavX Angle", navX.getAngle());
    
    Logger.recordOutput("Real States", getStates());
    Logger.recordOutput("Pose", getPose());

    SmartDashboard.putBoolean("FIELD-RELATIVE?", fieldRelativeStatus);
  }
}
