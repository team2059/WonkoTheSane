package org.team2059.Wonko.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import org.team2059.Wonko.Constants;
import org.team2059.Wonko.util.SwerveUtilities;

/**
 * Swerve Drive Specialties MK4n swerve module
 * 2x REV Robotics NEO Vortex motors
 */
public class MK4nVortexModule implements SwerveModuleIO {
  private final SparkFlex driveMotor;
  private final SparkFlex azimuthMotor;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder azimuthEncoder;

  private final CANcoder canCoder;
  private final Rotation2d offset;

  private final PIDController azimuthController;

  public MK4nVortexModule(
    int driveMotorCanID,
    int azimuthMotorCanID,
    int canCoderCanID,
    double canCoderOffsetRadians,
    boolean isDriveInverted,
    boolean isAzimuthInverted
  ) {
    // Configure motor controllers
    driveMotor = new SparkFlex(driveMotorCanID, SparkLowLevel.MotorType.kBrushless);
    azimuthMotor = new SparkFlex(azimuthMotorCanID, SparkLowLevel.MotorType.kBrushless);

    configureSpark(
      driveMotor,
      isDriveInverted,
      Constants.DrivetrainConstants.driveEncoderPositionConversionFactor,
      Constants.DrivetrainConstants.driveEncoderVelocityConversionFactor
    );

    configureSpark(
      azimuthMotor,
      isAzimuthInverted,
      Constants.DrivetrainConstants.rotationEncoderPositionConversionFactor,
      Constants.DrivetrainConstants.rotationEncoderVelocityConversionFactor
    );

    // Clear any sticky faults for debugging
    driveMotor.clearFaults();
    azimuthMotor.clearFaults();

    // Configure PID controller
    azimuthController = new PIDController(Constants.DrivetrainConstants.kPRotation, 0, 0);
    azimuthController.enableContinuousInput(-Math.PI, Math.PI);
    azimuthController.setTolerance(Units.degreesToRadians(1));

    // Configure cancoder, the absolute encoder of the module
    canCoder = new CANcoder(canCoderCanID);
    offset = new Rotation2d(canCoderOffsetRadians);
    configureCanCoder();

    // Get built-in relative encoders
    driveEncoder = driveMotor.getEncoder();
    azimuthEncoder = azimuthMotor.getEncoder();
  }

  /**
   * Configure a Spark motor controller.
   * In 2025, REV made changes requiring use
   * of a Spark[Max/Flex]Config object
   *
   * @param spark                    The Spark to configure
   * @param inverted                 Boolean motor inversion value
   * @param positionConversionFactor MotorRotations x [This factor] = units
   * @param velocityConversionFactor MotorRotations x [This factor] = units/sec
   */
  private void configureSpark(
    SparkFlex spark,
    boolean inverted,
    double positionConversionFactor,
    double velocityConversionFactor
  ) {
    SparkFlexConfig config = new SparkFlexConfig();

    config
      .inverted(inverted)
      .idleMode(SparkBaseConfig.IdleMode.kBrake);

    config.encoder
      .positionConversionFactor(positionConversionFactor)
      .velocityConversionFactor(velocityConversionFactor);

    spark.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }

  /**
   * Configure cancoder to operate with necessary behavior
   * [0,1) wrap range, CCW+ direction
   */
  private void configureCanCoder() {
    // Create the new configuration
    CANcoderConfiguration config = new CANcoderConfiguration();

    // Makes the range of the sensor 0-1 so that radians can be calculated
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;

    // Makes turning ccw positive
    config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

    // Apply cancer configuration
    canCoder.getConfigurator().apply(config);
  }

  /**
   * Set rotation encoder position to the offset of the CANcoder magnet
   */
  public void initRotationOffset() {
    azimuthEncoder.setPosition(getCANcoderRad().getRadians());
  }

  /**
   * @return Current position of drive motor (meters)
   */
  public double getDriveEncoderPosition() {
    return driveEncoder.getPosition();
  }

  /**
   * @return Rotation2d of current rotation encoder position (radians), range 0-2pi
   */
  public Rotation2d getRotationEncoderPosition() {
    return new Rotation2d(azimuthEncoder.getPosition());
  }

  /**
   * @return Current velocity of drive motor (m/s)
   */
  public double getDriveVelocity() {
    return driveEncoder.getVelocity();
  }

  /**
   * @return Current velocity of rotation motor (rad/s)
   */
  public double getRotationVelocity() {
    return azimuthEncoder.getVelocity();
  }

  /**
   * @return Rotation2d of absolute position from cancoder
   */
  public Rotation2d getCANcoderRad() {
    double canCoderRad = (Math.PI * 2 * canCoder.getAbsolutePosition().getValueAsDouble()) - offset.getRadians() % (2 * Math.PI);
    return new Rotation2d(canCoderRad);
  }

  public double getDriveVolts() {
    return (driveMotor.getAppliedOutput() * driveMotor.getBusVoltage());
  }

  public double getRotationVolts() {
    return (azimuthMotor.getAppliedOutput() * azimuthMotor.getBusVoltage());
  }

  public double getDriveCurrent() {
    return driveMotor.getOutputCurrent();
  }

  public double getRotationCurrent() {
    return azimuthMotor.getOutputCurrent();
  }

  public double getDriveMotorTemp() {
    return driveMotor.getMotorTemperature();
  }

  public double getRotationMotorTemp() {
    return azimuthMotor.getMotorTemperature();
  }

  /**
   * Reset Spark builtin encoders.
   * DriveEncoder = 0, RotationEncoder = cancoder offset
   */
  @Override
  public void resetEncoders() {
    driveEncoder.setPosition(0);
    azimuthEncoder.setPosition(getCANcoderRad().getRadians());
  }

  /**
   * @return Current SwerveModuleState of a module
   */
  @Override
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), getCANcoderRad());
  }

  /**
   * Set the state of a module
   * @param state containing linear velocity setpoint and angular setpoint
   * @param isClosedLoop
   */
  @Override
  public void setState(SwerveModuleState state, boolean isClosedLoop) {
    // Deadband
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }

    // Optimize angle of state to minimize rotation magnitude
    state = SwerveUtilities.optimize(state, getCANcoderRad());

    // PID-controlled rotation
    azimuthMotor.set(azimuthController.calculate(getCANcoderRad().getRadians(), state.angle.getRadians()));

    //   if (isClosedLoop) {
    //     // Feedforward-controlled translation
    //     driveMotor.setVoltage(DrivetrainConstants.driveFF.calculate(state.speedMetersPerSecond));
    //   } else {
    //     // Direct set, won't be as accurate
    //     driveMotor.set(state.speedMetersPerSecond / DrivetrainConstants.maxVelocity);
    //   }
    driveMotor.setVoltage(Constants.DrivetrainConstants.driveFF.calculate(state.speedMetersPerSecond));
  }

  @Override
  public void setAzimuthAngle(double angleRadians) {
    azimuthMotor.set(azimuthController.calculate(getCANcoderRad().getRadians(), angleRadians));
  }

  /**
   * Stop all motors in a module
   */
  @Override
  public void stop() {
    driveMotor.set(0);
    azimuthMotor.set(0);
  }

  @Override
  public void updateInputs(SwerveModuleIOInputs inputs) {

    inputs.drivePosition = getDriveEncoderPosition();
    inputs.driveVelocity = getDriveVelocity();

    inputs.driveAppliedVolts = getDriveVolts();
    inputs.driveCurrentAmps = getDriveCurrent();

    inputs.azimuthAbsolutePosition = getCANcoderRad().getRadians();
    inputs.azimuthPosition = getRotationEncoderPosition().getRadians();
    inputs.azimuthVelocity = getRotationVelocity();

    inputs.azimuthAppliedVolts = getRotationVolts();
    inputs.azimuthCurrentAmps = getRotationCurrent();

    inputs.driveMotorTemp = getDriveMotorTemp();
    inputs.azimuthMotorTemp = getRotationMotorTemp();
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveMotor.setVoltage(volts);
  }

  @Override
  public void setAzimuthVoltage(double volts) {
    azimuthMotor.setVoltage(volts);
  }
}