// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.studica.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveConstants;

public class Drivetrain extends SubsystemBase {

  public static boolean fieldRelativeStatus = true;

  /**
   * SwerveModule objects
   * Parameters:
   * - drive motor CAN ID
   * - rotation motor CAN ID
   * - external CANcoder CAN ID
   * - measured CANcoder offset in radians
   */
  private final SwerveModule frontLeft = new SwerveModule(
    SwerveConstants.frontLeftDriveMotorId, 
    SwerveConstants.frontLeftRotationMotorId, 
    SwerveConstants.frontLeftCanCoderId, 
    SwerveConstants.frontLeftOffsetRad
  );

  private final SwerveModule frontRight = new SwerveModule(
    SwerveConstants.frontRightDriveMotorId, 
    SwerveConstants.frontRightRotationMotorId, 
    SwerveConstants.frontRightCanCoderId, 
    SwerveConstants.frontRightOffsetRad
  );

  private final SwerveModule backLeft = new SwerveModule(
    SwerveConstants.backLeftDriveMotorId, 
    SwerveConstants.backLeftRotationMotorId, 
    SwerveConstants.backLeftCanCoderId, 
    SwerveConstants.backLeftOffsetRad
  );

  private final SwerveModule backRight = new SwerveModule(
    SwerveConstants.backRightDriveMotorId, 
    SwerveConstants.backRightRotationMotorId, 
    SwerveConstants.backRightCanCoderId, 
    SwerveConstants.backRightOffsetRad
  );

  // Create NavX object (gyro)
  private final AHRS navX;

  // Create swerve drive odometry engine, used to track robot on field
  private final SwerveDriveOdometry odometry;

  // Kinematics gets each module relative to its center
  private final SwerveDriveKinematics kinematics;

  private Field2d field = new Field2d();

  public Drivetrain() {

    // NavX may need an extra second to start...
    navX = new AHRS(AHRS.NavXComType.kMXP_SPI);
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        navX.reset();
      } catch (Exception e) {
      }
    }).start();

    // Set rotation encoder position to the absolute position of the cancoder
    frontLeft.syncRotationEncoders();
    frontRight.syncRotationEncoders();
    backLeft.syncRotationEncoders();
    backRight.syncRotationEncoders();

    // drive motor inversions: offsets mess with these sometimes
    frontLeft.setMotorInversion(frontLeft.getDriveMotor(), true);
    frontRight.setMotorInversion(frontRight.getDriveMotor(), false);
    backLeft.setMotorInversion(backLeft.getDriveMotor(), true);
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

    kinematics = new SwerveDriveKinematics(
      SwerveConstants.flModuleOffset,
      SwerveConstants.frModuleOffset,
      SwerveConstants.blModuleOffset,
      SwerveConstants.brModuleOffset
    );

    odometry = new SwerveDriveOdometry(
      kinematics, 
      navX.getRotation2d(), 
      getModulePositions()
    );

    configureAutoBuilder();

    // Set up custom logging to add the urrent path to a field 2d widget
    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
  
    SmartDashboard.putData("Field", field);
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
  public void resetPose(Pose2d pose) {
    odometry.resetPosition(navX.getRotation2d(), getModulePositions(), pose);
  }

  /**
   * @return ChassisSpeeds of current robot-relative speeds
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(getStates());
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
    return navX.getRotation2d();
  }

  /**
   * @return current swerve module positions in SwerveModulePosition[] array
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = {
      new SwerveModulePosition(frontLeft.getDriveDistance(), frontLeft.getCanCoderRad()),
      new SwerveModulePosition(frontRight.getDriveDistance(), frontRight.getCanCoderRad()),
      new SwerveModulePosition(backLeft.getDriveDistance(), backLeft.getCanCoderRad()),
      new SwerveModulePosition(backRight.getDriveDistance(), backRight.getCanCoderRad())
    };

    return positions;
  }

  /**
   * Method to drive robot-relative
   * @param chassisSpeeds
   */
  public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
    setModuleStates(
      kinematics.toSwerveModuleStates(targetSpeeds)
    );
  }

  /**
   * Method to drive field-relative
   * @param chassisSpeeds
   */
  public void driveFieldRelative(ChassisSpeeds chassisSpeeds) {
    driveRobotRelative(
      ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, getPose().getRotation())
    );
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
    // makes it never go above specified max speed
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.maxVelocity);
    
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
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.maxVelocity);

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
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();

      // Configure AutoBuilder
      AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier, MUST be robot relative 
        (speeds) -> driveRobotRelative(speeds), // Method that will drive the robot given robot-relative chassisspeeds 
        new PPHolonomicDriveController(
          AutoConstants.autoTranslationConstants, // Translation PID constants 
          AutoConstants.autoRotationConstants), // Rotation PID constants
        config, // Robot configuration
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
      DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(getHeading(), getModulePositions());
    field.setRobotPose(getPose());
    
    // Shove everything logging-related here
    Logger.recordOutput("Real States", getStates());
    Logger.recordOutput("Field-relative?", fieldRelativeStatus);
  }

  /* =================== */
  /* SWERVE MODULE CLASS */
  /* =================== */

  private class SwerveModule {
    private final SparkMax driveMotor;
    private final SparkMax rotationMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder rotationEncoder;

    private final CANcoder canCoder;
    private final double offset;

    private final PIDController rotationPidController;

    SwerveModule(
      int driveMotorId,
      int rotationMotorId,
      int canCoderId,
      double canCoderOffsetRadians
    ) {
      // Configure motor controllers
      driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
      rotationMotor = new SparkMax(rotationMotorId, MotorType.kBrushless);

      configureSpark(
        driveMotor,
        false,
        IdleMode.kBrake,
        SwerveConstants.driveEncoderPositionConversionFactor,
        SwerveConstants.driveEncoderVelocityConversionFactor
      );

      configureSpark(
        rotationMotor,
        false,
        IdleMode.kBrake,
        SwerveConstants.rotationEncoderPositionConversionFactor,
        SwerveConstants.rotationEncoderVelocityConversionFactor
      );

      // Configure cancoder
      canCoder = new CANcoder(canCoderId);
      offset = canCoderOffsetRadians;
      configureCanCoder();

      // use PID for turning, for accurate and fast setpoint approach
      rotationPidController = new PIDController(SwerveConstants.kPRotation, 0, 0);

      driveEncoder = driveMotor.getEncoder();
      rotationEncoder = rotationMotor.getEncoder();
    }

    /**
     * Configure a Spark motor controller.
     * In 2025, REV made changes that now require
     * use of a Spark[Max/Flex]Config object
     */
    void configureSpark(
      SparkMax spark,
      boolean inverted,
      IdleMode idleMode,
      double positionConversionFactor,
      double velocityConversionFactor
    ) {
      SparkMaxConfig config = new SparkMaxConfig();

      config
        .inverted(inverted)
        .idleMode(idleMode);

      config.encoder
        .positionConversionFactor(positionConversionFactor)
        .velocityConversionFactor(velocityConversionFactor);

      spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Configure cancoder to operate with necessary behavior
     * [0,1) wrap range, CCW+ direction
     */
    void configureCanCoder() {
      // Create the new configuration
      CANcoderConfiguration config = new CANcoderConfiguration();

      // Make the range of the sensor 0-1 so that radians can be calculated
      config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;

      // CCW+ rotation is the standard
      config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

      // Apply the configuration
      canCoder.getConfigurator().apply(config);
    }

    /**
     * Set rotation encoder position to the offset of the CANcoder magnet
     */
    void syncRotationEncoders() {
      rotationEncoder.setPosition(getCanCoderRad().getRadians());
    }

    /**
     * @return current distance traveled by drive motor (meters)
     */
    double getDriveDistance() {
      return driveEncoder.getPosition() * (SwerveConstants.wheelDiameter / 2.0);
    }

    /**
     * @return current distance of rotation motor (radians)
     */
    double getRotationDistance() {
      return rotationEncoder.getPosition();
    }

    /**
     * @return current velocity of drive motor (meters/sec)
     */
    double getDriveVelocity() {
      return driveEncoder.getVelocity();
    }

    /**
     * @return current velocity of rotation motor (radians/sec)
     */
    double getRotationVelocity() {
      return rotationEncoder.getVelocity();
    }

    /**
     * @return SparkMax drive motor object
     */
    SparkMax getDriveMotor() {
      return driveMotor;
    }

    /**
     * @return SparkMax rotation motor object
     */
    SparkMax getRotationMotor() {
      return rotationMotor;
    }

    /**
     * @return CANcoder object
     */
    CANcoder getCanCoder() {
      return canCoder;
    }

    /**
     * Set a Spark motor to be inverted.
     * @param motor Spark to be inverted
     * @param inverted boolean containing inversion value
     */
    void setMotorInversion(SparkMax motor, boolean inverted) {
      motor.configure(
        new SparkMaxConfig().inverted(inverted), 
        ResetMode.kResetSafeParameters, 
        PersistMode.kPersistParameters
      );
    }

    /**
     * @return absolute position from cancoder
     */
    Rotation2d getCanCoderRad() {
      return new Rotation2d(
        ((Math.PI * canCoder.getAbsolutePosition().getValueAsDouble()) - offset) % (2 * Math.PI)
      );
    }

    /**
     * Reset Spark builtin encoders.
     * Drive = 0; Rotation = cancoder
     */
    public void resetEncoders() {
      driveEncoder.setPosition(0);
      rotationEncoder.setPosition(getCanCoderRad().getRadians());
    }

    /**
     * @return Current SwerveModuleState
     */
    SwerveModuleState getState() {
      return new SwerveModuleState(getDriveVelocity(), getCanCoderRad());
    }

    static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
      double lowerBound;
      double upperBound;
      double lowerOffset = scopeReference % (2.0 * Math.PI);
      if (lowerOffset >= 0) {
        lowerBound = scopeReference - lowerOffset;
        upperBound = scopeReference + ((2.0 * Math.PI) - lowerOffset);
      } else {
        upperBound = scopeReference - lowerOffset;
        lowerBound = scopeReference - ((2.0 * Math.PI) + lowerOffset);
      }
      while (newAngle < lowerBound) {
        newAngle += (2.0 * Math.PI);
      }
      while (newAngle > upperBound) {
        newAngle -= (2.0 * Math.PI);
      }
      if (newAngle - scopeReference > (Math.PI)) {
        newAngle -= (2.0 * Math.PI);
      } else if (newAngle - scopeReference < -(Math.PI)) {
        newAngle += (2.0 * Math.PI);
      }
      return newAngle;
    }

    /**
     * Minimize the change in heading the desired swerve module state would require
     * by potentially
     * reversing the direction the wheel spins. Customized from WPILib's version to
     * include placing in
     * appropriate scope for CTRE and REV onboard control as both controllers as of
     * writing don't have
     * support for continuous input.
     *
     * @param desiredState The desired state.
     * @param currentAngle The current module angle.
     */
    static SwerveModuleState optimize(
      SwerveModuleState desiredState, Rotation2d currentAngle) {

      double targetAngle = placeInAppropriate0To360Scope(currentAngle.getRadians(), desiredState.angle.getRadians());

      double targetSpeed = desiredState.speedMetersPerSecond;
      double delta = (targetAngle - currentAngle.getRadians());
      if (Math.abs(delta) > (Math.PI / 2)) {
        targetSpeed = -targetSpeed;
        targetAngle = delta > Math.PI / 2 ? (targetAngle -= Math.PI) : (targetAngle += Math.PI);
      }
      return new SwerveModuleState(targetSpeed, new Rotation2d(targetAngle));
    }

    /**
     * Set desired SwerveModuleState.
     * Drive: open-loop, Rotation: closed-loop
     * 
     * @param state desired SwerveModuleState
     */
    void setDesiredState(SwerveModuleState state) {
      // Find the closest equivalent angle to target
      state = optimize(state, getCanCoderRad());

      // Set drive motor speed to the ratio of target speed to max speed
      driveMotor.set(state.speedMetersPerSecond / SwerveConstants.maxVelocity);

      // use PID for turning to avoid overshooting
      rotationMotor.set(rotationPidController.calculate(getCanCoderRad().getRadians(), state.angle.getRadians()));
    }

    /**
     * Set desired SwerveModuleState.
     * Drive: FF, Rotation: PID
     * 
     * @param desiredState desired SwerveModuleState
     */
    void setDesiredStateClosedLoop(SwerveModuleState desiredState) {
      // Deadband
      if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
          stop();
          return;
      }

      // Create optimized state to work with
      SwerveModuleState optimizedState = optimize(desiredState, getCanCoderRad());

      // Set outputs (PID for rotation, FF for drive)
      rotationMotor.set(rotationPidController.calculate(
          getCanCoderRad().getRadians(), // current angle
          optimizedState.angle.getRadians() // target angle
      ));
      driveMotor.setVoltage(
        SwerveConstants.driveFF.calculate(optimizedState.speedMetersPerSecond)
      );
    }

    /**
     * Stop all motors in a module
     */
    void stop() {
      driveMotor.set(0);
      rotationMotor.set(0);
    }
  }
}
