package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveBaseConstants;
import frc.robot.Constants.VisionConstants;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveBase extends SubsystemBase {

  /**
   * Subsystem that controls the drivetrain of the robot
   * Handles all the odometry and base movement for the chassis
   */

  /**
   * absolute encoder offsets for the wheels
   * 180 degrees added to offset values to invert one side of the robot so that it
   * doesn't spin in place
   */

  /**
   * SwerveModule objects
   * Parameters:
   * drive motor can ID
   * rotation motor can ID
   * external CANCoder can ID
   * measured CANCoder offset
   */

  private final SwerveModule frontLeft = new SwerveModule(0,
      SwerveBaseConstants.frontLeftDriveMotorId,
      SwerveBaseConstants.frontLeftRotationMotorId,
      SwerveBaseConstants.frontLeftRotationEncoderId,
      SwerveBaseConstants.frontLeftAngleOffset);

  private final SwerveModule frontRight = new SwerveModule(1,
      SwerveBaseConstants.frontRightDriveMotorId,
      SwerveBaseConstants.frontRightRotationMotorId,
      SwerveBaseConstants.frontRightRotationEncoderId,
      SwerveBaseConstants.frontRightAngleOffset);

  private final SwerveModule rearLeft = new SwerveModule(2,
      SwerveBaseConstants.rearLeftDriveMotorId,
      SwerveBaseConstants.rearLeftRotationMotorId,
      SwerveBaseConstants.rearLeftRotationEncoderId,
      SwerveBaseConstants.rearLeftAngleOffset);

  private final SwerveModule rearRight = new SwerveModule(3,
      SwerveBaseConstants.rearRightDriveMotorId,
      SwerveBaseConstants.rearRightRotationMotorId,
      SwerveBaseConstants.rearRightRotationEncoderId,
      SwerveBaseConstants.rearRightAngleOffset);

  private final SwerveModule[] modules = new SwerveModule[] { frontLeft, frontRight, rearLeft, rearRight };

  private final AHRS navX;

  private final SwerveDrivePoseEstimator poseEstimator;
  private Vision vision = Vision.getInstance();

  /**
   * odometry for the robot, measured in meters for linear motion and radians for
   * rotational motion
   * Takes in kinematics and robot angle for parameters
   */
  // private final SwerveDriveOdometry odometry = new
  // SwerveDriveOdometry(SwerveBaseConstants.kinematics, new Rotation2d(),
  // getModulePositions());

  public SwerveBase() {

    navX = new AHRS(SPI.Port.kMXP);
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        navX.reset();
      } catch (Exception e) {
      }
    }).start();

    // initialize the rotation offsets for the CANCoders
    frontLeft.initRotationOffset();
    frontRight.initRotationOffset();
    rearLeft.initRotationOffset();
    rearRight.initRotationOffset();

    // reset the measured distance driven for each module
    frontLeft.resetDistance();
    frontRight.resetDistance();
    rearLeft.resetDistance();
    rearRight.resetDistance();

    rearRight.getDriveMotor().setInverted(false);
    rearLeft.getDriveMotor().setInverted(true);
    frontRight.getDriveMotor().setInverted(false);
    frontLeft.getDriveMotor().setInverted(true);

    rearRight.getRotationMotor().setInverted(true);
    rearLeft.getRotationMotor().setInverted(true);
    frontRight.getRotationMotor().setInverted(true);
    frontLeft.getRotationMotor().setInverted(true);

    configureAutoBuilder();

    poseEstimator = new SwerveDrivePoseEstimator(SwerveBaseConstants.kinematics,
        getHeading(), getModulePositions(),
        new Pose2d(0, 0, navX.getRotation2d()), VisionConstants.STATE_STDS, VisionConstants.VISION_STDS);
    // poseEstimator = new SwerveDrivePoseEstimator(
    // SwerveBaseConstants.kinematics,
    // getHeading(),
    // getModulePositions(), new Pose2d(0, 0, navX.getRotation2d()));

  }

  @Override
  public void periodic() {

    // update the odometry every 20ms
    // odometry.update(getHeading(), getModulePositions());

    // Add vision to pose estimator
    final Optional<EstimatedRobotPose> optionalEstimatedPose = vision
        .getEstimatedGlobalPose(poseEstimator.getEstimatedPosition());
    // if (optionalEstimatedPose.isPresent() && RobotState.isTeleop()) {
    // final EstimatedRobotPose estimatedPose = optionalEstimatedPose.get();
    // poseEstimator.addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(),
    // estimatedPose.timestampSeconds);

    // }

    if (optionalEstimatedPose.isPresent()) {
      final EstimatedRobotPose estimatedPose = optionalEstimatedPose.get();
      poseEstimator.addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(),
          estimatedPose.timestampSeconds);

    }

    // Update the odometry of the swerve drive using the wheel encoders and gyro in
    // the periodic block every 20 ms
    poseEstimator.update(
        getHeading(), getModulePositions());

    // if (Math.abs(poseEstimator.getEstimatedPosition().getRotation().getDegrees())
    // >= 178) {
    // zeroHeading();
    // }

    Logger.recordOutput("Current Pose", getPose());

    SmartDashboard.putString("Robot pose",
        getPose().toString());
    SmartDashboard.putNumber("navX Heading",
        getHeading().getDegrees());

    // SmartDashboard.putNumber("yaw",
    // navX.getYaw());

    // SmartDashboard.putNumber("pitch",
    // navX.getPitch());

    // for (SwerveModule module : modules) {
    // SmartDashboard.putNumber(module.getModuleID() + "velocity setpoint",
    // module.velolictySetpoint);
    // SmartDashboard.putNumber(module.getModuleID() + "actual velocity",
    // module.currentDriveVelocity);
    // SmartDashboard.putNumber(module.getModuleID() + "angular setpoint",
    // module.angularSetpoint);
    // SmartDashboard.putNumber(module.getModuleID() + "actual angle",
    // module.actualAngle);

    // SmartDashboard.putNumber(module.getModuleID() + "actual CAN angle",
    // module.getCanCoderAngle().getRotations());
    // }

    // SmartDashboard.putNumber("distance to speaker",
    // vision.getDistancetoSpeaker(poseEstimator.getEstimatedPosition()));

  }

  public void configureAutoBuilder() {

    // Configure the AutoBuilder last
    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveFieldRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(SwerveBaseConstants.translationkP, 0.0, 0.0), // Translation PID constants
            new PIDConstants(SwerveBaseConstants.rotationkP, 0.0, 0.0), // Rotation PID constants
            SwerveBaseConstants.maxSpeed, // Max module speed, in m/s
            SwerveBaseConstants.driveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest
                                                 // module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);

  }

  public Command pathFindToPose(Pose2d pose, PathConstraints constraints, double goalEndVel) {
    return AutoBuilder.pathfindToPose(pose, constraints, goalEndVel);
  }

  /**
   * Drives the robot robot-relative according to provided {@link ChassisSpeeds}.
   * 
   * @param chassisSpeeds The desired ChassisSpeeds. Should be robot relative.
   */
  public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
    SwerveModuleState[] newStates = Constants.SwerveBaseConstants.kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(newStates, Constants.SwerveBaseConstants.maxSpeed);
    // commanded.set(newStates);
    setModuleStates(newStates);

  }

  public void driveFieldRelative(ChassisSpeeds chassisSpeeds) {
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
    discreteSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(discreteSpeeds, getHeading());
    SwerveModuleState[] newStates = Constants.SwerveBaseConstants.kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(newStates, Constants.SwerveBaseConstants.maxSpeed);
    setModuleStates(newStates);

  }

  /**
   * Gets robot relative ChassisSpeeds.
   * 
   * @return The robot-relative {@link ChassisSpeeds}.
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        Constants.SwerveBaseConstants.kinematics.toChassisSpeeds(getStates()),
        getHeading());

    return chassisSpeeds;
  }

  /**
   * method for driving the robot
   * Parameters:
   * forward linear value
   * sideways linear value
   * rotation value
   * if the control is field relative or robot relative
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
        ? ChassisSpeeds.fromFieldRelativeSpeeds(
            forward, strafe, rotation, getHeading())
        : new ChassisSpeeds(forward, strafe, rotation);

    speeds = ChassisSpeeds.discretize(speeds, 0.02);

    // use kinematics (wheel placements) to convert overall robot state to array of
    // individual module states
    SwerveModuleState[] states = SwerveBaseConstants.kinematics.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveBaseConstants.maxSpeed);

    setModuleStates(states);

  }

  /**
   * Method to set the desired state for each swerve module
   * Uses PID and feedforward control to control the linear and rotational values
   * for the modules
   */
  public void setModuleStates(SwerveModuleState[] moduleStates) {

    SwerveModuleState[] optimizedModuleStates = new SwerveModuleState[4];
    optimizedModuleStates[0] = SwerveModule.optimize(frontLeft.getState(), frontLeft.getIntegratedAngle());
    optimizedModuleStates[1] = SwerveModule.optimize(frontRight.getState(), frontRight.getIntegratedAngle());
    optimizedModuleStates[2] = SwerveModule.optimize(rearLeft.getState(), rearLeft.getIntegratedAngle());
    optimizedModuleStates[3] = SwerveModule.optimize(rearRight.getState(), rearRight.getIntegratedAngle());

    frontLeft.setDesiredStateClosedLoop(moduleStates[0]);
    frontRight.setDesiredStateClosedLoop(moduleStates[1]);
    rearLeft.setDesiredStateClosedLoop(moduleStates[2]);
    rearRight.setDesiredStateClosedLoop(moduleStates[3]);

  }

  /**
   * Gets states of the four swerve modules.
   * 
   * @return The states of the four swerve modules in a {@link SwerveModuleState}
   *         array.
   */
  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule module : modules) {
      states[module.getModuleID()] = module.getState();
    }

    return states;
  }

  // returns an array of SwerveModulePositions
  public SwerveModulePosition[] getModulePositions() {

    SwerveModulePosition[] positions = {
        new SwerveModulePosition(-frontLeft.getCurrentDistanceMetersPerSecond(), frontLeft.getIntegratedAngle()),
        new SwerveModulePosition(-frontRight.getCurrentDistanceMetersPerSecond(), frontRight.getIntegratedAngle()),
        new SwerveModulePosition(-rearLeft.getCurrentDistanceMetersPerSecond(), rearLeft.getIntegratedAngle()),
        new SwerveModulePosition(-rearRight.getCurrentDistanceMetersPerSecond(), rearRight.getIntegratedAngle())

    };

    return positions;

  }

  /**
   * Return the current position of the robot on field
   * Based on drive encoder and gyro reading
   */
  public Pose2d getPose() {

    // return odometry.getPoseMeters();

    return poseEstimator.getEstimatedPosition();

  }

  // reset the current pose to a desired pose
  public void resetOdometry(Pose2d pose) {

    // odometry.resetPosition(getHeading(), getModulePositions(), pose);

    poseEstimator.resetPosition(getHeading(), getModulePositions(), pose);

  }

  // reset the measured distance driven for each module
  public void resetDriveDistances() {

    frontLeft.resetDistance();
    frontRight.resetDistance();
    rearLeft.resetDistance();
    rearRight.resetDistance();

  }

  // get the current heading of the robot based on the gyro
  public Rotation2d getHeading() {

    return Rotation2d.fromDegrees(-navX.getYaw());

  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    navX.reset();

    // var pose = this.poseEstimator.getEstimatedPosition();
    // pose = pose.rotateBy(navX.getRotation2d().times(-1));
  }

  public AHRS getNavX() {
    return navX;
  }

  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    rearRight.stop();
    rearLeft.stop();
  }

}