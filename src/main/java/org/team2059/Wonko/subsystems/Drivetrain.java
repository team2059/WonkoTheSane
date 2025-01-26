// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2059.Wonko.subsystems;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import org.littletonrobotics.junction.Logger;
import org.team2059.Wonko.Constants;
import org.team2059.Wonko.Constants.AutoConstants;
import org.team2059.Wonko.Constants.DrivetrainConstants;

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
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Drivetrain extends SubsystemBase {

  public static boolean fieldRelativeStatus = true;

  // Create 4 SwerveModule objects using given constants.
  private final SwerveModule frontLeft = new SwerveModule(
    DrivetrainConstants.frontLeftDriveMotorId, 
    DrivetrainConstants.frontLeftRotationMotorId, 
    DrivetrainConstants.frontLeftCanCoderId, 
    DrivetrainConstants.frontLeftOffsetRad,
    false,
    true);
  private final SwerveModule frontRight = new SwerveModule(
    DrivetrainConstants.frontRightDriveMotorId, 
    DrivetrainConstants.frontRightRotationMotorId, 
    DrivetrainConstants.frontRightCanCoderId, 
    DrivetrainConstants.frontRightOffsetRad,
    true,
    true);
  private final SwerveModule backLeft = new SwerveModule(
    DrivetrainConstants.backLeftDriveMotorId, 
    DrivetrainConstants.backLeftRotationMotorId, 
    DrivetrainConstants.backLeftCanCoderId, 
    DrivetrainConstants.backLeftOffsetRad,
    false,
    true);
  private final SwerveModule backRight = new SwerveModule(
    DrivetrainConstants.backRightDriveMotorId, 
    DrivetrainConstants.backRightRotationMotorId, 
    DrivetrainConstants.backRightCanCoderId, 
    DrivetrainConstants.backRightOffsetRad,
    true,
    true);

  // Create NavX object (gyro)
  private final AHRS navX;

  // Create swerve drive odometry engine, used to track robot on field
  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(Constants.DrivetrainConstants.kinematics, new Rotation2d(), getModulePositions());

  private final SysIdRoutine sysIDDriveRoutine;

  // Mutable holders for unit-safe voltage, linear distance, and linear velocity values, persisted to avoid reallocation.
  private final MutVoltage driveRoutineAppliedVoltage = Volts.mutable(0);
  private final MutDistance driveRoutineDistance = Meters.mutable(0);
  private final MutLinearVelocity driveRoutineVelocity = MetersPerSecond.mutable(0);

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

    // SysID characterization configuration
    sysIDDriveRoutine = new SysIdRoutine(
      // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage
      new SysIdRoutine.Config(), 
      new SysIdRoutine.Mechanism(
        // Tell SysID how to plumb the driving voltage to the motors
        voltage -> {
          frontLeft.setDriveVoltage(voltage.in(Volts));
          frontRight.setDriveVoltage(voltage.in(Volts));
          backLeft.setDriveVoltage(voltage.in(Volts));
          backRight.setDriveVoltage(voltage.in(Volts));
        }, 
        // Tell SysID how to record a frame of data for each motor on the mechanism
        log -> {
          log.motor("drive-frontleft")
            .voltage(driveRoutineAppliedVoltage.mut_replace(frontLeft.getDriveMotor().get() * RobotController.getBatteryVoltage(), Volts))
            .linearPosition(driveRoutineDistance.mut_replace(frontLeft.getDriveEncoderPosition(), Meters))
            .linearVelocity(driveRoutineVelocity.mut_replace(frontLeft.getDriveVelocity(), MetersPerSecond));

          log.motor("drive-frontright")
            .voltage(driveRoutineAppliedVoltage.mut_replace(frontRight.getDriveMotor().get() * RobotController.getBatteryVoltage(), Volts))
            .linearPosition(driveRoutineDistance.mut_replace(frontRight.getDriveEncoderPosition(), Meters))
            .linearVelocity(driveRoutineVelocity.mut_replace(frontRight.getDriveVelocity(), MetersPerSecond));

          log.motor("drive-backleft")
            .voltage(driveRoutineAppliedVoltage.mut_replace(backLeft.getDriveMotor().get() * RobotController.getBatteryVoltage(), Volts))
            .linearPosition(driveRoutineDistance.mut_replace(backLeft.getDriveEncoderPosition(), Meters))
            .linearVelocity(driveRoutineVelocity.mut_replace(backLeft.getDriveVelocity(), MetersPerSecond));

          log.motor("drive-backright")
            .voltage(driveRoutineAppliedVoltage.mut_replace(backRight.getDriveMotor().get() * RobotController.getBatteryVoltage(), Volts))
            .linearPosition(driveRoutineDistance.mut_replace(backRight.getDriveEncoderPosition(), Meters))
            .linearVelocity(driveRoutineVelocity.mut_replace(backRight.getDriveVelocity(), MetersPerSecond));
        }, 
        // Tell SysId to make generated commands require this subsystem, suffix test state in
        // WPILog with this subsystem's name ("drive")
        this
      )
    );
  }

  // Returns a command that will execute a quasistatic test in the given direction
  public Command sysIdDriveQuasistatic(SysIdRoutine.Direction direction) {
    return sysIDDriveRoutine.quasistatic(direction);
  }

  // Returns a command that will execute a dynamic test in the given direction
  public Command sysIdDriveDynamic(SysIdRoutine.Direction direction) {
    return sysIDDriveRoutine.dynamic(direction);
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
    // makes it never go above specified max velocity
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DrivetrainConstants.maxVelocity);
    // Sets the speed and rotation of each module
    frontLeft.setState(desiredStates[0], false);
    frontRight.setState(desiredStates[1], false);
    backLeft.setState(desiredStates[2], false);
    backRight.setState(desiredStates[3], false);

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
    odometry.update(getHeading(), getModulePositions());    

    Logger.recordOutput("Pose", getPose());

  }
}
