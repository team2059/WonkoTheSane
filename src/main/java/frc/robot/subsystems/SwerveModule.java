package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule extends SubsystemBase {
    private final SparkMax driveMotor;
    private final SparkMax rotationMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder rotationEncoder;

    private final CANcoder canCoder;
    private final Rotation2d offset;

    private final PIDController rotationPidController;

    public SwerveModule(
        int driveMotorId,
        int rotationMotorId,
        int canCoderId,
        double canCoderOffsetRadians
    ) {
        // Instantiate motor controller objects
        driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        rotationMotor = new SparkMax(rotationMotorId, MotorType.kBrushless);

        // Configure motor controllers
        configureSparkMax(
          driveMotor,
          false,
          IdleMode.kBrake,
          SwerveConstants.driveEncoderPositionConversionFactor,
          SwerveConstants.driveEncoderVelocityConversionFactor
        );

        configureSparkMax(
          rotationMotor,
          false,
          IdleMode.kBrake,
          SwerveConstants.rotationEncoderPositionConversionFactor,
          SwerveConstants.rotationEncoderVelocityConversionFactor
        );

        // Instantiate rotation PID controller, for smoother and more accurate rotation
        rotationPidController = new PIDController(SwerveConstants.kPTurning, 0, 0);

        // tells pidcontroller that -pi is the same as +pi, can calculate shorter path to setpoint from either sign
        rotationPidController.enableContinuousInput(-Math.PI, Math.PI);        

        // Instantiate new CANcoder and respective offset, set configuration
        canCoder = new CANcoder(canCoderId);
        offset = new Rotation2d(canCoderOffsetRadians);

        configureCanCoder();

        // Set encoder objects to appropriate motor's encoders
        driveEncoder = driveMotor.getEncoder();
        rotationEncoder = rotationMotor.getEncoder();
    }

    private void configureSparkMax(
        SparkMax max, 
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

      max.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configureCanCoder() {
        // Create the new configuration
        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();

        // Makes the range of the sensor 0-1 so that radians can be calculated
        canCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;

        // Makes turning ccw positive
        canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        // Apply cancoder configuration
        canCoder.getConfigurator().apply(canCoderConfig);
    }

    public void initRotationOffset() {
        rotationEncoder.setPosition(getCANcoderRad().getRadians());
    }

    public double getDriveEncoderPosition() {
        return driveEncoder.getPosition();
    }

    public Rotation2d getRotationEncoderPosition() {
        double unsignedAngle = rotationEncoder.getPosition() % (2 * Math.PI);

        if (unsignedAngle < 0) unsignedAngle += 2 * Math.PI;

        return new Rotation2d(unsignedAngle);
    }

    public CANcoder getCANcoder() {
        return canCoder;
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getRotationVelocity() {
        return rotationEncoder.getVelocity();
    }

    public SparkMax getDriveMotor() {
        return driveMotor;
    }

    public SparkMax getRotationMotor() {
        return rotationMotor;
    }

    public void setMotorInversion(SparkMax motor, boolean inverted) {
      motor.configure(
        new SparkMaxConfig()
          .inverted(inverted), 
        ResetMode.kResetSafeParameters, 
        PersistMode.kPersistParameters
      );
    }

    // Gets actual amount of rotation from each motor by getting cancoder position offset (from center)
    // and subtracting from current rotation
    public Rotation2d getCANcoderRad() {
        double canCoderRad = (Math.PI * 2 * canCoder.getAbsolutePosition().getValueAsDouble()) - offset.getRadians() % (2 * Math.PI);
        return new Rotation2d(canCoderRad);
    }

    // Resets encoders and makes the rotation motor equal to the offset
    // so that we account for the offset
    public void resetEncoders() {
        driveEncoder.setPosition(0);
        rotationEncoder.setPosition(getCANcoderRad().getRadians());
    }

    // Takes in velocity and angle which calculates how much it needs to turn and apply forward motion.
    // This will come in use throughout other code
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), getCANcoderRad());
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

    // Actually applies a SwerveModuleState, but uses scaling rather than PID for the drive motor
    public void setDesiredStates(SwerveModuleState state) {
        // Optimize finds the closest angle to the target
        state = optimize(state, getRotationEncoderPosition());

        driveMotor.set(state.speedMetersPerSecond / SwerveConstants.maxVelocity);

        // use PID for turning to avoid overshooting
        rotationMotor.set(rotationPidController.calculate(getRotationEncoderPosition().getRadians(), state.angle.getRadians()));
    }

    /**
     * Method to set the desired state of a swerve module,
     * using PID and feedforward to control the output
     * 
     * @param desiredState SwerveModuleState object that holds desired linear and rotational setpoint
     */
    public void setDesiredStateClosedLoop(SwerveModuleState desiredState) {
        // Deadband
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        // Create optimized state to work with
        SwerveModuleState optimizedState = optimize(desiredState, getIntegratedAngle());

        // Set outputs (PID for rotation, FF for drive)
        rotationMotor.set(rotationPidController.calculate(
            getIntegratedAngle().getRadians(), // current angle
            optimizedState.angle.getRadians() // target angle
        ));
        driveMotor.setVoltage(SwerveConstants.driveFF.calculate(
            optimizedState.speedMetersPerSecond // target speed
        ));
    }

    public double getCurrentDistanceMetersPerSecond() {
        return driveEncoder.getPosition() * (SwerveConstants.wheelDiameter / 2.0);
    }

    public Rotation2d getIntegratedAngle() {

        double unsignedAngle = rotationEncoder.getPosition() % (2 * Math.PI);
    
        if (unsignedAngle < 0)
          unsignedAngle += 2 * Math.PI;
    
        return new Rotation2d(unsignedAngle);
    
    }

    public void stop() {
        driveMotor.set(0);
        rotationMotor.set(0);
    }

    @Override
    public void periodic() {
    }
  
    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }
}


