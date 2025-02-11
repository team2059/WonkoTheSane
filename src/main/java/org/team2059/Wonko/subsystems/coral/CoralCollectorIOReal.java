package org.team2059.Wonko.subsystems.coral;

import org.team2059.Wonko.Constants.CoralCollectorConstants;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class CoralCollectorIOReal implements CoralCollectorIO {
    private SparkFlex intakeMotor;
    private SparkMax tiltMotor;

    // Through bore encoder is a DutyCycleEncoder, goes from 0 - 1
    private DutyCycleEncoder tiltEncoder;
    
    // IR sensor - detects whether coral is present by light signal on or off
    private DigitalInput irSensor;

    public CoralCollectorIOReal() {
        intakeMotor = new SparkFlex(CoralCollectorConstants.intakeMotorId, MotorType.kBrushless);
        tiltMotor = new SparkMax(CoralCollectorConstants.tiltMotorId, MotorType.kBrushless);

        intakeMotor.configure(
            new SparkFlexConfig()  
                .inverted(false)
                .idleMode(IdleMode.kBrake),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        tiltMotor.configure(
            new SparkMaxConfig()
                .inverted(false)
                .idleMode(IdleMode.kBrake),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        tiltEncoder = new DutyCycleEncoder(CoralCollectorConstants.thruBoreDio);
        tiltEncoder.setDutyCycleRange(CoralCollectorConstants.tiltEncoderMin, CoralCollectorConstants.tiltEncoderMax);

        irSensor = new DigitalInput(CoralCollectorConstants.irSensorDio);
    }

    @Override
    public void updateInputs(CoralCollectorIOInputs inputs) {
        inputs.intakeMotorAppliedVolts = (intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage());
        inputs.tiltMotorAppliedVolts = (tiltMotor.getAppliedOutput() * tiltMotor.getBusVoltage());
        inputs.intakeMotorCurrentAmps = intakeMotor.getOutputCurrent();
        inputs.tiltMotorCurrentAmps = tiltMotor.getOutputCurrent();
        inputs.intakeMotorTemp = intakeMotor.getMotorTemperature();
        inputs.tiltMotorTemp = tiltMotor.getMotorTemperature();
        inputs.thruBoreConnected = tiltEncoder.isConnected();
        inputs.thruBorePos = tiltEncoder.get();
        inputs.hasCoral = irSensor.get();
    }

    @Override
    public void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }

    @Override
    public void setTiltPosition(double position) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setTiltPosition'");
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
}