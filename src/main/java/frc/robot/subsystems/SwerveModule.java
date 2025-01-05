package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveModuleConstants;

public class SwerveModule extends SubsystemBase {

  /**
   * Class to represent and handle a swerve module
   * A module's state is measured by a CANCoder for the absolute position,
   * integrated CANEncoder for relative position
   * for both rotation and linear movement
   */

  public PIDController rotationController;
  private final int moduleID;

  public static double DrivePIDOutput = 0;
  public static double feedForwardOutputVoltage = 0;
  public static double driveOutput = 0;
  public static double velolictySetpoint = 0;
  public static double currentDriveVelocity = 0;

  public static double angularSetpoint = 0;
  public static double actualAngle = 0;

  private final CANSparkMax driveMotor;
  private final CANSparkMax rotationMotor;

  private final PIDController driveController = new PIDController(SwerveModuleConstants.drivekP, 0, 0);

  public CANSparkMax getDriveMotor() {
    return driveMotor;
  }

  public CANSparkMax getRotationMotor() {
    return rotationMotor;
  }

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder rotationEncoder;

  private final CANcoder canCoder;

  // absolute offset for the CANCoder so that the wheels can be aligned when the
  // robot is turned on
  private final Rotation2d offset;

  public SwerveModule(int moduleID,
      int driveMotorId,
      int rotationMotorId,
      int canCoderId,
      double measuredOffsetRadians) {

    driveMotor = new CANSparkMax(driveMotorId, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    rotationMotor = new CANSparkMax(rotationMotorId, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);

    driveEncoder = driveMotor.getEncoder();
    rotationEncoder = rotationMotor.getEncoder();

    rotationController = new PIDController(SwerveModuleConstants.rotationkP, 0, 0.0);
    rotationController.enableContinuousInput(-Math.PI, Math.PI);

    canCoder = new CANcoder(canCoderId);

    offset = new Rotation2d(measuredOffsetRadians);

    driveMotor.setIdleMode(IdleMode.kBrake);
    rotationMotor.setIdleMode(IdleMode.kBrake);

    this.moduleID = moduleID;

    // set the output of the drive encoder to be in radians for linear measurement
    // driveEncoder.setPositionConversionFactor(
    // 2.0 * Math.PI / Swerve.driveGearRatio);
    driveEncoder.setPositionConversionFactor(SwerveModuleConstants.driveEncoderPositionConversionFactor);

    // set the output of the drive encoder to be in radians per second for velocity
    // measurement
    // driveEncoder.setVelocityConversionFactor(
    // 2.0 * Math.PI / 60 / Swerve.driveGearRatio);
    driveEncoder.setVelocityConversionFactor(SwerveModuleConstants.driveEncoderPositionConversionFactor / 60.0);

    // set the output of the rotation encoder to be in radians
    // rotationEncoder.setPositionConversionFactor(2 * Math.PI /
    // Swerve.angleGearRatio);
    rotationEncoder.setPositionConversionFactor(SwerveModuleConstants.rotationEncoderPositionConversionFactor);

    // configure the CANCoder to output in unsigned (wrap around from 360 to 0
    // degrees)
    // canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    canCoder.getConfigurator().apply(config);

    driveMotor.burnFlash();
    rotationMotor.burnFlash();

  }

  public void resetDistance() {

    driveEncoder.setPosition(0.0);

  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getCurrentVelocityMetersPerSecond(), getCanCoderAngle());
  }

  public double getDriveDistanceRadians() {

    return driveEncoder.getPosition();

  }

  public Rotation2d getCanCoderAngle() {

    // double unsignedAngle =
    // (Units.degreesToRadians(canCoder.getAbsolutePosition()) -
    // offset.getRadians())
    // % (2 * Math.PI);

    double unsignedAngle = (Math.PI * 2 *
        canCoder.getAbsolutePosition().getValueAsDouble()) - offset.getRadians()
            % (2 * Math.PI);

    return new Rotation2d(unsignedAngle);

  }

  public Rotation2d getIntegratedAngle() {

    double unsignedAngle = rotationEncoder.getPosition() % (2 * Math.PI);

    if (unsignedAngle < 0)
      unsignedAngle += 2 * Math.PI;

    return new Rotation2d(unsignedAngle);

  }

  // public double getCurrentVelocityRadiansPerSecond() {

  // return driveEncoder.getVelocity();

  // }

  public double getCurrentVelocityMetersPerSecond() {

    // return driveEncoder.getVelocity() * (Swerve.wheelDiameter / 2.0);
    return driveEncoder.getVelocity();

  }

  public double getCurrentDistanceMetersPerSecond() {
    return driveEncoder.getPosition();
    // return driveEncoder.getPosition() * (Swerve.wheelDiameter / 2.0);
  }

  public static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
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
  public static SwerveModuleState optimize(
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
   * Returns the module ID.
   *
   * @return The ID number of the module (0-3).
   */
  public int getModuleID() {
    return moduleID;
  }

  // initialize the integrated NEO encoder to the offset (relative to home
  // position)
  // measured by the CANCoder
  public void initRotationOffset() {

    rotationEncoder.setPosition(getCanCoderAngle().getRadians());

  }

  /**
   * Method to set the desired state of the swerve module
   * Parameter:
   * SwerveModuleState object that holds a desired linear and rotational setpoint
   * Uses PID and a feedforward to control the output
   */
  public void setDesiredStateClosedLoop(SwerveModuleState unoptimizedDesiredState) {
    if (Math.abs(unoptimizedDesiredState.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }

    SwerveModuleState optimizedDesiredState = optimize(unoptimizedDesiredState, getIntegratedAngle());

    angularSetpoint = optimizedDesiredState.angle.getRadians();

    actualAngle = getIntegratedAngle().getRadians();

    velolictySetpoint = optimizedDesiredState.speedMetersPerSecond;

    currentDriveVelocity = getCurrentVelocityMetersPerSecond();

    // DrivePIDOutput = driveController.calculate(currentDriveVelocity,
    // velolictySetpoint);
    feedForwardOutputVoltage = (SwerveModuleConstants.driveFF.calculate(velolictySetpoint));
    // driveOutput = (DrivePIDOutput + feedForwardOutputVoltage);

    rotationMotor.set(rotationController.calculate(actualAngle, angularSetpoint));
    // driveMotor.setVoltage(-driveOutput);
    driveMotor.setVoltage(-feedForwardOutputVoltage);

  }

  public void resetEncoders() {

    driveEncoder.setPosition(0);
    rotationEncoder.setPosition(0);

  }

  public void stop() {
    driveMotor.set(0);
    rotationMotor.set(0);
  }
}