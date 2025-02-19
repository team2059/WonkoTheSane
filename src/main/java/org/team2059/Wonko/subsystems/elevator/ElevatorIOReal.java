package org.team2059.Wonko.subsystems.elevator;

import org.team2059.Wonko.Constants.ElevatorConstants;
import org.team2059.Wonko.util.LoggedTunableNumber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj.DigitalInput;

public class ElevatorIOReal implements ElevatorIO {

    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private SparkMaxConfig config = new SparkMaxConfig();
    private SparkClosedLoopController controller;
    
    // Limit switches
    // private final DigitalInput[] limitSwitches = new DigitalInput[5];

    private LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", 0.0);
    private LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/kI", 0.0);
    private LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", 0.0);

    public ElevatorIOReal() {
        motor = new SparkMax(ElevatorConstants.motorId, MotorType.kBrushless);

        config
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40)
            .closedLoopRampRate(0.5);
        
        config.encoder
            .positionConversionFactor(ElevatorConstants.positionConversionFactor)
            .velocityConversionFactor(ElevatorConstants.velocityConversionFactor);

        config.closedLoop
            .pid(kP.get(), kI.get(), kD.get())
            .outputRange(-0.75, 0.75)
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = motor.getEncoder();
        controller = motor.getClosedLoopController();

        // for (int i = 0; i < limitSwitches.length; i++) {
        //     limitSwitches[i] = new DigitalInput(ElevatorConstants.limitSwitchDIO[i]);
        // }
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {

        // Update tunables
        if (
            kP.hasChanged(hashCode())
            || kI.hasChanged(hashCode())
            || kD.hasChanged(hashCode())
        ) {
            SparkMaxConfig pidConfigChange = new SparkMaxConfig();
            pidConfigChange.closedLoop.pid(kP.get(), kI.get(), kD.get());
            motor.configure(pidConfigChange, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }

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

    @Override
    public void setSpeed(double speed) {
        motor.set(speed);
    }

    @Override
    public void setPositionClosedLoopWithFF(double position, double arbFF) {
        controller.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0, arbFF);
    }
}
