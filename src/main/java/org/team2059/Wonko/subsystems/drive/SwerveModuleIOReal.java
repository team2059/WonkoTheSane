package org.team2059.Wonko.subsystems.drive;

import org.littletonrobotics.junction.Logger;
import org.team2059.Wonko.Constants.DrivetrainConstants;
import org.team2059.Wonko.util.SwerveUtilities;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

public class SwerveModuleIOReal implements SwerveModuleIO {
    private final SparkFlex driveMotor;
    private final SparkFlex rotationMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder rotationEncoder;

    private final CANcoder canCoder;
    private final Rotation2d offset;

    private final PIDController rotationPidController;

    private final PIDController drivePidController;

    private final SimpleMotorFeedforward driveFF;

    public SwerveModuleIOReal(
        int driveMotorId,
        int rotationMotorId,
        int canCoderId,
        double canCoderOffsetRadians,
        boolean isDriveInverted,
        boolean isRotationInverted,
        double kS,
        double kV,
        double kA,
        double kP
    ) {
        // Motor controllers
        driveMotor = new SparkFlex(driveMotorId, MotorType.kBrushless);
        rotationMotor = new SparkFlex(rotationMotorId, MotorType.kBrushless);

        // Configure motor controllers
        configureSpark(
            driveMotor,
            isDriveInverted,
            IdleMode.kBrake,
            DrivetrainConstants.driveEncoderPositionConversionFactor,
            DrivetrainConstants.driveEncoderVelocityConversionFactor
        );

        configureSpark(
            rotationMotor,
            isRotationInverted,
            IdleMode.kBrake,
            DrivetrainConstants.rotationEncoderPositionConversionFactor,
            DrivetrainConstants.rotationEncoderVelocityConversionFactor
        );

        driveMotor.clearFaults();
        rotationMotor.clearFaults();

        rotationPidController = new PIDController(DrivetrainConstants.kPRotation, 0, 0);
        rotationPidController.enableContinuousInput(-Math.PI, Math.PI);

        rotationPidController.setTolerance(Units.degreesToRadians(1));

        drivePidController = new PIDController(kP, 0, 0);

        driveFF = new SimpleMotorFeedforward(kS, kV, kA);

        canCoder = new CANcoder(canCoderId);
        offset = new Rotation2d(canCoderOffsetRadians);
        configureCanCoder();

        driveEncoder = driveMotor.getEncoder();
        rotationEncoder = rotationMotor.getEncoder();
    }

    /**
     * Configure a Spark motor controller. 
     * In 2025, REV made changes requiring use 
     * of a Spark[Max/Flex]Config object
     * 
     * @param spark The Spark to configure
     * @param inverted Boolean motor inversion value
     * @param idleMode IdleMode.kBrake or IdleMode.kCoast
     * @param positionConversionFactor MotorRotations x [This factor] = units
     * @param velocityConversionFactor MotorRotations x [This factor] = units/sec
     */
    private void configureSpark(
        SparkFlex spark, 
        boolean inverted, 
        IdleMode idleMode,
        double positionConversionFactor,
        double velocityConversionFactor
    ) {
      SparkFlexConfig config = new SparkFlexConfig();

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
    private void configureCanCoder() {
        // Create the new configuration
        CANcoderConfiguration config = new CANcoderConfiguration();

        // Makes the range of the sensor 0-1 so that radians can be calculated
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;

        // Makes rotationing ccw positive
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        // Apply cancoder configuration
        canCoder.getConfigurator().apply(config);
    }

    /**
     * Set rotation encoder position to the offset of the CANcoder magnet
     */
    public void initRotationOffset() {
        rotationEncoder.setPosition(getCANcoderRad().getRadians());
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
        return new Rotation2d(rotationEncoder.getPosition());
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
        return rotationEncoder.getVelocity();
    }

    /**
     * @return SparkFlex drive motor object
     */
    public SparkFlex getDriveMotor() {
        return driveMotor;
    }

    /**
     * @return SparkFlex rotation motor object
     */
    public SparkFlex getRotationMotor() {
        return rotationMotor;
    }

    /**
     * @return CANcoder object
     */
    public CANcoder getCANcoder() {
        return canCoder;
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
        return (rotationMotor.getAppliedOutput() * rotationMotor.getBusVoltage());
    }

    public double getDriveCurrent() {
        return driveMotor.getOutputCurrent();
    }

    public double getRotationCurrent() {
        return rotationMotor.getOutputCurrent();
    }

    public double getDriveMotorTemp() {
        return driveMotor.getMotorTemperature();
    }

    public double getRotationMotorTemp() {
        return rotationMotor.getMotorTemperature();
    }

    /**
     * Reset Spark builtin encoders.
     * DriveEncoder = 0, RotationEncoder = cancoder offset
     */
    @Override
    public void resetEncoders() {
        driveEncoder.setPosition(0);
        rotationEncoder.setPosition(getCANcoderRad().getRadians());
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
      rotationMotor.set(rotationPidController.calculate(getCANcoderRad().getRadians(), state.angle.getRadians()));

    //   if (isClosedLoop) {
    //     // Feedforward-controlled translation
    //     driveMotor.setVoltage(DrivetrainConstants.driveFF.calculate(state.speedMetersPerSecond));
    //   } else {
    //     // Direct set, won't be as accurate
    //     driveMotor.set(state.speedMetersPerSecond / DrivetrainConstants.maxVelocity);
    //   }
      driveMotor.setVoltage(DrivetrainConstants.driveFF.calculate(state.speedMetersPerSecond) + drivePidController.calculate(state.speedMetersPerSecond));
    }

    @Override
    public void setRotationMotorAnglePID(double angleRadians) {
        rotationMotor.set(rotationPidController.calculate(getCANcoderRad().getRadians(), angleRadians));
    }

    /**
     * Stop all motors in a module
     */
    @Override
    public void stop() {
        driveMotor.set(0);
        rotationMotor.set(0);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        
        inputs.drivePosition = getDriveEncoderPosition();
        inputs.driveVelocity = getDriveVelocity();

        inputs.driveAppliedVolts = getDriveVolts();
        inputs.driveCurrentAmps = getDriveCurrent();

        inputs.rotationAbsolutePositionRadians = getCANcoderRad().getRadians();
        inputs.rotationPositionRadians = getRotationEncoderPosition().getRadians();
        inputs.rotationVelocityRadPerSec = getRotationVelocity();

        inputs.rotationAppliedVolts = getRotationVolts();
        inputs.rotationCurrentAmps = getRotationCurrent();

        inputs.driveMotorTemp = getDriveMotorTemp();
        inputs.rotationMotorTemp = getRotationMotorTemp();

        inputs.rotationPidAtSetpoint = rotationPidController.atSetpoint();
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveMotor.setVoltage(volts);
    }

    @Override
    public void setRotationVoltage(double volts) {
        rotationMotor.setVoltage(volts);
    }
}