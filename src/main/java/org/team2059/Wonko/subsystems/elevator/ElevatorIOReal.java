package org.team2059.Wonko.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

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
import edu.wpi.first.wpilibj.DigitalInput;

public class ElevatorIOReal implements ElevatorIO {

    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private SparkClosedLoopController controller;
    
    // Limit switches
    private DigitalInput zeroLimit;

    private LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", ElevatorConstants.kP);
    private LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/kI", ElevatorConstants.kI);
    private LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", ElevatorConstants.kD);

    public ElevatorIOReal() {
        motor = new SparkMax(ElevatorConstants.motorId, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit((int)ElevatorConstants.currentLimit.in(Amps))
            .closedLoopRampRate(0.5);
        config.encoder
            .positionConversionFactor(ElevatorConstants.positionConversionFactor)
            .velocityConversionFactor(ElevatorConstants.velocityConversionFactor);
        config.closedLoop
            .pid(kP.get(), kI.get(), kD.get())
            .outputRange(-1.0, 1.0)
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = motor.getEncoder();
        controller = motor.getClosedLoopController();

        zeroLimit = new DigitalInput(0);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {

        // Update tunables if changed
        LoggedTunableNumber.ifChanged(
            hashCode(), 
            () -> {
                SparkMaxConfig pidConfigChange = new SparkMaxConfig();
                pidConfigChange.closedLoop.pid(kP.get(), kI.get(), kD.get());
                motor.configure(pidConfigChange, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            }, 
            kP, kI, kD
        );

        // Update all logged input values
        inputs.positionMeters = encoder.getPosition();
        inputs.velocityMetersPerSecond = encoder.getVelocity();
        inputs.appliedVolts = getAppliedVolts();
        inputs.currentAmps = motor.getOutputCurrent();
        inputs.motorTemp = motor.getMotorTemperature();

        inputs.zeroLimit = !zeroLimit.get();
    }

    // local helper method
    public double getAppliedVolts() {
        return (motor.getAppliedOutput() * motor.getBusVoltage());
    }

    @Override
    public void stop() {
        motor.setVoltage(0);
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
