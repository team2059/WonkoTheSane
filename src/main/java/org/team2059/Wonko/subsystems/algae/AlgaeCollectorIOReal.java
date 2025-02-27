package org.team2059.Wonko.subsystems.algae;

import org.team2059.Wonko.Constants.AlgaeCollectorConstants;
import org.team2059.Wonko.util.LoggedTunableNumber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
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
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class AlgaeCollectorIOReal implements AlgaeCollectorIO {
    private SparkFlex motor1;
    private SparkFlex motor2;

    private SparkFlex tiltMotor;

    private DutyCycleEncoder tiltEncoder;
    
    private RelativeEncoder tiltMotorIntegratedEncoder;

    private LoggedTunableNumber kP = new LoggedTunableNumber("AlgaeCollector/Tilt/kP", 0.5);
    private LoggedTunableNumber kI = new LoggedTunableNumber("AlgaeCollector/Tilt/kI", 0.0);
    private LoggedTunableNumber kD = new LoggedTunableNumber("AlgaeCollector/Tilt/kD", 0.0);

    private SparkClosedLoopController tiltController;

    public AlgaeCollectorIOReal() {
        // Create intake and tilt motors and configure them
        motor1 = new SparkFlex(AlgaeCollectorConstants.motor1Id, MotorType.kBrushless);
        motor2 = new SparkFlex(AlgaeCollectorConstants.motor2Id, MotorType.kBrushless);
        tiltMotor = new SparkFlex(AlgaeCollectorConstants.tiltMotorId, MotorType.kBrushless);

        SparkFlexConfig motor1Config = new SparkFlexConfig();
        motor1Config
            .inverted(false)
            .idleMode(IdleMode.kBrake);
        motor1.configure(motor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motor1.clearFaults();

        SparkFlexConfig motor2Config = new SparkFlexConfig();
        motor2Config
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        motor2.configure(motor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motor2.clearFaults();

        SparkFlexConfig tiltConfig = new SparkFlexConfig();
        tiltConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake);
        tiltConfig.encoder
            .positionConversionFactor(AlgaeCollectorConstants.tiltMotorPositionConvFactor)
            .velocityConversionFactor(AlgaeCollectorConstants.tiltMotorVelocityConvFactor);
        tiltConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(kP.get(), kI.get(), kD.get())
            .outputRange(-1, 1);
        tiltMotor.configure(tiltConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        tiltMotor.clearFaults();

        // Create thru-bore encoder
        tiltEncoder = new DutyCycleEncoder(AlgaeCollectorConstants.tiltEncoderDio);
        tiltEncoder.setInverted(false);

        tiltController = tiltMotor.getClosedLoopController();

        tiltMotorIntegratedEncoder = tiltMotor.getEncoder();

        // Through my testing I found that the thrubore reports the wrong angle
        // until a few seconds have passed since power up.
        // new Thread(() -> {
        //     try {
        //         Thread.sleep(2500);
        //         double initialPos = 2.0 * Math.PI * tiltEncoder.get();
        //         tiltMotorIntegratedEncoder.setPosition(initialPos);
        //         System.out.println("ALG INITIAL POS:"+ initialPos);
        //     } catch (Exception e) {
        //         e.printStackTrace();
        //     }
        //   }).start();
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

        inputs.motor1AppliedVolts = (motor1.getAppliedOutput() * motor1.getBusVoltage());
        inputs.motor2AppliedVolts = (motor2.getAppliedOutput() * motor2.getBusVoltage());
        inputs.tiltMotorAppliedVolts = (tiltMotor.getAppliedOutput() * tiltMotor.getBusVoltage());

        inputs.motor1CurrentAmps = motor1.getOutputCurrent();
        inputs.motor2CurrentAmps = motor2.getOutputCurrent();
        inputs.tiltMotorCurrentAmps = tiltMotor.getOutputCurrent();

        inputs.motor1Temp = motor1.getMotorTemperature();
        inputs.motor2Temp = motor2.getMotorTemperature();
        inputs.tiltMotorTemp = tiltMotor.getMotorTemperature();

        inputs.thruBoreConnected = tiltEncoder.isConnected();
        inputs.thruBorePositionRadians = tiltEncoder.get() * 2.0 * Math.PI + AlgaeCollectorConstants.horizontalOffset;

        if (Math.abs(inputs.thruBorePositionRadians - tiltMotorIntegratedEncoder.getPosition()) >= 0.01) {
            tiltMotorIntegratedEncoder.setPosition(inputs.thruBorePositionRadians);
        }

        inputs.integratedTiltPosRadians = tiltMotorIntegratedEncoder.getPosition();
        inputs.integratedTiltVelRadPerSec = tiltMotorIntegratedEncoder.getVelocity();

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
    public void setTiltPos(double posRadians, double arbFF) {
        tiltController.setReference(
            posRadians,
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            arbFF
        );
    }
}
