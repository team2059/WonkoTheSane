package org.team2059.Wonko.subsystems.elevator;

import org.team2059.Wonko.Constants.ElevatorConstants;
import org.team2059.Wonko.util.LoggedTunableNumber;
import org.team2059.Wonko.util.SparkConfigurationUtility;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
// import edu.wpi.first.wpilibj.DigitalInput;

public class ElevatorIOVortex implements ElevatorIO {

    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private SparkClosedLoopController pidController;
    private ElevatorFeedforward feedforward;
    
    // Limit switches
    // private final DigitalInput[] limitSwitches = new DigitalInput[5];

    // Tunable numbers
    private LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", 0.1);
    private LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/kI", 0);
    private LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", 0);

    public ElevatorIOVortex() {
        motor = new SparkMax(ElevatorConstants.motorId, MotorType.kBrushless);

        feedforward = new ElevatorFeedforward(0, 0, 0);
        
        SparkConfigurationUtility.configureSpark(
            motor,
            true,
            IdleMode.kBrake,
            ElevatorConstants.positionConversionFactor,
            ElevatorConstants.velocityConversionFactor
        );

        encoder = motor.getEncoder();
        pidController = motor.getClosedLoopController();

        SparkConfigurationUtility.setPID(motor, kP.get(), kI.get(), kD.get());

        // for (int i = 0; i < limitSwitches.length; i++) {
        //     limitSwitches[i] = new DigitalInput(ElevatorConstants.limitSwitchDIO[i]);
        // }
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        // Update tunables
        if (kP.hasChanged(hashCode()) || kI.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
            SparkConfigurationUtility.setPID(motor, kP.get(), kI.get(), kD.get());
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
    public void setLevel(int level) {
        pidController.setReference(
            ElevatorConstants.levels[level], 
            ControlType.kPosition
        );
    }

    @Override
    public void setPosition(double setpointPositionMeters) {
        pidController.setReference(
            setpointPositionMeters, 
            ControlType.kPosition
        );
    }

    @Override
    public void setVelocity(double velocitySetpointMps) {
        pidController.setReference(
            velocitySetpointMps, 
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0,
            feedforward.calculate(velocitySetpointMps)
        );
    }
    
}
