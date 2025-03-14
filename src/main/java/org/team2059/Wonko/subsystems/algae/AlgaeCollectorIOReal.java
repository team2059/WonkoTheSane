package org.team2059.Wonko.subsystems.algae;

import org.team2059.Wonko.Constants.AlgaeCollectorConstants;
import org.team2059.Wonko.util.LoggedTunableNumber;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;

public class AlgaeCollectorIOReal implements AlgaeCollectorIO {
    private SparkFlex motor1;
    private SparkFlex motor2;

    private SparkFlex tiltMotor;
    
    private AbsoluteEncoder tiltAbsoluteEnc;

    private LoggedTunableNumber kP = new LoggedTunableNumber("AlgaeCollector/Tilt/kP", 0.5);
    private LoggedTunableNumber kI = new LoggedTunableNumber("AlgaeCollector/Tilt/kI", 0.0);
    private LoggedTunableNumber kD = new LoggedTunableNumber("AlgaeCollector/Tilt/kD", 0.0);

    private SparkClosedLoopController tiltController;

    private DigitalInput irSensor;

    public AlgaeCollectorIOReal() {
        // Create intake and tilt motors and configure them
        motor1 = new SparkFlex(AlgaeCollectorConstants.motor1Id, MotorType.kBrushless);
        motor2 = new SparkFlex(AlgaeCollectorConstants.motor2Id, MotorType.kBrushless);
        tiltMotor = new SparkFlex(AlgaeCollectorConstants.tiltMotorId, MotorType.kBrushless);

        // Configure intake motor 1
        SparkFlexConfig motor1Config = new SparkFlexConfig();
        motor1Config
            .inverted(false)
            .idleMode(IdleMode.kBrake);
        motor1.configure(motor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motor1.clearFaults();

        // Configure intake motor 2
        SparkFlexConfig motor2Config = new SparkFlexConfig();
        motor2Config
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        motor2.configure(motor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motor2.clearFaults();

        // Configure tilt motor
        SparkFlexConfig tiltConfig = new SparkFlexConfig();
        tiltConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .closedLoopRampRate(0.75);
        tiltConfig.absoluteEncoder
            .zeroOffset(0.38)
            .inverted(false)
            .positionConversionFactor(2.0 * Math.PI)
            .velocityConversionFactor(2.0 * Math.PI / 60)
            .zeroCentered(true);
        tiltConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(kP.get(), kI.get(), kD.get())
            .outputRange(-1, 1);
        tiltMotor.configure(tiltConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        tiltAbsoluteEnc = tiltMotor.getAbsoluteEncoder();

        tiltController = tiltMotor.getClosedLoopController();

        irSensor = new DigitalInput(AlgaeCollectorConstants.irSensorDio);
    }

    @Override
    public void setIntakeSpeed(double speed) {
        motor1.set(speed);
        motor2.set(speed);
    }

    @Override
    public void stopIntake() {
        motor1.setVoltage(0);
        motor2.setVoltage(0);
    }

    @Override
    public void stopTilt() {
        tiltMotor.setVoltage(0);
    }

    @Override
    public void stopAll() {
        stopTilt();
        stopIntake();
    }

    @Override
    public void updateInputs(AlgaeCollectorIOInputs inputs) {

        // Update tunables
        LoggedTunableNumber.ifChanged(
            hashCode(), 
            () -> {
                SparkFlexConfig tempTiltConfig = new SparkFlexConfig();
                tempTiltConfig.closedLoop.pid(kP.get(), kI.get(), kD.get());
                tiltMotor.configure(tempTiltConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            }, 
            kP, kI, kD
        );

        inputs.motor1AppliedVolts = (motor1.getAppliedOutput() * motor1.getBusVoltage());
        inputs.motor2AppliedVolts = (motor2.getAppliedOutput() * motor2.getBusVoltage());
        inputs.tiltMotorAppliedVolts = (tiltMotor.getAppliedOutput() * tiltMotor.getBusVoltage());

        inputs.motor1CurrentAmps = motor1.getOutputCurrent();
        inputs.motor2CurrentAmps = motor2.getOutputCurrent();
        inputs.tiltMotorCurrentAmps = tiltMotor.getOutputCurrent();

        inputs.motor1Temp = motor1.getMotorTemperature();
        inputs.motor2Temp = motor2.getMotorTemperature();
        inputs.tiltMotorTemp = tiltMotor.getMotorTemperature();

        inputs.tiltAbsPosRadians = tiltAbsoluteEnc.getPosition();
        inputs.tiltMotorVelocityRadPerSec = tiltAbsoluteEnc.getVelocity();

        inputs.hasAlgae = !irSensor.get();
    }

    @Override
    public void setTiltSpeed(double speed) {
       tiltMotor.set(speed);
    }

    @Override
    public void setTiltVolts(double volts) {
        tiltMotor.setVoltage(MathUtil.clamp(volts, -12, 12));
    }

    @Override
    public void setTiltPos(double posRadians) {
        // System.out.println("Target CoralTiltPos: " + posRadians);
        tiltController.setReference(
            posRadians, 
            ControlType.kPosition
        );
    }
}
