package org.team2059.Wonko.subsystems.elevator;

import org.team2059.Wonko.Constants.ElevatorConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj.DigitalInput;

public class ElevatorIOReal implements ElevatorIO {

    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private SparkMaxConfig config = new SparkMaxConfig();
    
    // Limit switches
    // private final DigitalInput[] limitSwitches = new DigitalInput[5];

    public ElevatorIOReal() {
        motor = new SparkMax(ElevatorConstants.motorId, MotorType.kBrushless);

        config
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        
        config.encoder
            .positionConversionFactor(ElevatorConstants.positionConversionFactor)
            .velocityConversionFactor(ElevatorConstants.velocityConversionFactor);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = motor.getEncoder();

        // for (int i = 0; i < limitSwitches.length; i++) {
        //     limitSwitches[i] = new DigitalInput(ElevatorConstants.limitSwitchDIO[i]);
        // }
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {

        // Update all logged input values
        inputs.positionMeters = encoder.getPosition();
        inputs.velocityMetersPerSecond = encoder.getPosition();
        inputs.appliedVolts = getAppliedVolts();
        inputs.currentAmps = motor.getOutputCurrent();
        inputs.motorTemp = motor.getMotorTemperature();

        // for (int i = 0; i < inputs.limitSwitches.length; i++) {
        //     inputs.limitSwitches[i] = limitSwitches[i].get();
        // }
    }

    public double getAppliedVolts() {
        return (motor.getAppliedOutput() * motor.getBusVoltage());
    }

    @Override
    public void stop() {
        motor.set(0);
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(MathUtil.clamp(volts, -12, 12));
    }

    @Override
    public void resetEncoder() {
        encoder.setPosition(0);
    }
}
