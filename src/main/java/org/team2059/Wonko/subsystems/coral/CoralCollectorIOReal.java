package org.team2059.Wonko.subsystems.coral;

import org.team2059.Wonko.Constants.CoralCollectorConstants;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class CoralCollectorIOReal implements CoralCollectorIO {
    private SparkFlex intakeMotor;
    private SparkFlex tiltMotor;

    // Through bore encoder is a DutyCycleEncoder, goes from 0 - 1
    private DutyCycleEncoder tiltEncoder;
    
    // IR sensor - detects whether coral is present by light signal on or off
    private DigitalInput irSensor;

    public CoralCollectorIOReal() {
        intakeMotor = new SparkFlex(CoralCollectorConstants.intakeMotorId, MotorType.kBrushless);
        tiltMotor = new SparkFlex(CoralCollectorConstants.tiltMotorId, MotorType.kBrushless);

        // Configure both Spark motor controllers
        SparkFlexConfig intakeConfig = new SparkFlexConfig();
        intakeConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake);
        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeMotor.clearFaults(); 

        SparkFlexConfig tiltConfig = new SparkFlexConfig();
        tiltConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        tiltMotor.configure(tiltConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        tiltMotor.clearFaults();

        // Configure thru-bore encoder
        tiltEncoder = new DutyCycleEncoder(CoralCollectorConstants.thruBoreDio);
        tiltEncoder.setInverted(true);

        // Configure IR sensor - reports presence of coral
        irSensor = new DigitalInput(CoralCollectorConstants.irSensorDio);
    }

    @Override
    public void updateInputs(CoralCollectorIOInputs inputs) {
        inputs.intakeMotorAppliedVolts = (intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage());
        inputs.tiltMotorAppliedVolts = (tiltMotor.getAppliedOutput() * tiltMotor.getBusVoltage());
        inputs.intakeMotorCurrentAmps = intakeMotor.getOutputCurrent();
        inputs.tiltMotorCurrentAmps = tiltMotor.getOutputCurrent();
        inputs.intakeMotorTemp = intakeMotor.getMotorTemperature();
        inputs.tiltMotorPos = tiltMotor.getEncoder().getPosition();
        inputs.tiltMotorTemp = tiltMotor.getMotorTemperature();
        inputs.thruBoreConnected = tiltEncoder.isConnected();
        inputs.thruBorePositionDegrees = tiltEncoder.get() * 360;
        inputs.hasCoral = irSensor.get();
    }

    @Override
    public void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }

    @Override
    public void stopIntake() {
        intakeMotor.setVoltage(0);
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
    public void setTiltSpeed(double speed) {
        tiltMotor.set(speed);
    }

    @Override
    public void setTiltVolts(double volts) {
        tiltMotor.setVoltage(volts);
    }
}