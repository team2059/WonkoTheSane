package org.team2059.Wonko.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.urcl.URCL;
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
import edu.wpi.first.wpilibj.DigitalInput;

public class ElevatorIOReal implements ElevatorIO {

    private final SparkMax rightMotor;
    private final SparkMax leftMotor; 

    private final RelativeEncoder rightEncoder;
    private final RelativeEncoder leftEncoder;

    private SparkClosedLoopController controller;
    

    // Limit switches
    private DigitalInput zeroLimit;

    private LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", ElevatorConstants.kP);
    private LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/kI", ElevatorConstants.kI);
    private LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", ElevatorConstants.kD);

    public ElevatorIOReal() {
        rightMotor = new SparkMax(ElevatorConstants.rightMotorId, MotorType.kBrushless);
        leftMotor = new SparkMax(ElevatorConstants.leftMotorId, MotorType.kBrushless); 

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
        rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.clearFaults();

        // // Will make left motor follow right motor 
        // config.follow(ElevatorConstants.rightMotorId, true); 
        config.follow(ElevatorConstants.rightMotorId, true);
        leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftMotor.clearFaults();

        rightEncoder = rightMotor.getEncoder();
        controller = rightMotor.getClosedLoopController();

        leftEncoder = leftMotor.getEncoder();

        // Gets limit switch at bottom of elevator
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
                rightMotor.configure(pidConfigChange, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            }, 
            kP, kI, kD
        );

        // Update all logged input values
        inputs.velocityMetersPerSecond = rightEncoder.getVelocity();

        inputs.rightMotorAppliedVolts = getAppliedVolts(rightMotor);
        inputs.rightMotorCurrentAmps = rightMotor.getOutputCurrent();
        inputs.rightMotorTemp = rightMotor.getMotorTemperature();

        inputs.leftMotorAppliedVolts = getAppliedVolts(leftMotor);
        inputs.leftMotorCurrentAmps = leftMotor.getOutputCurrent();
        inputs.leftMotorTemp = leftMotor.getMotorTemperature();

        inputs.zeroLimit = !zeroLimit.get();

        if (inputs.zeroLimit) resetEncoder();
        inputs.positionMeters = rightEncoder.getPosition();

    }

    // local helper method
    public double getAppliedVolts(SparkMax motor) {
        return (motor.getAppliedOutput() * motor.getBusVoltage());
    }

    @Override
    public void stop() {
        // leftMotor.setVoltage(0);
        rightMotor.setVoltage(0);
    }

    @Override
    public void setVoltage(double volts) {
        // leftMotor.setVoltage(MathUtil.clamp(volts, -12, 12));
        rightMotor.setVoltage(MathUtil.clamp(volts, -12, 12));
    }

    @Override
    public void resetEncoder() {
        rightEncoder.setPosition(0);
        leftEncoder.setPosition(0);
    }

    @Override
    public void setSpeed(double speed) {
        // leftMotor.set(speed);
        rightMotor.set(speed);
    }

    @Override
    public void setPositionClosedLoopWithFF(double position, double arbFF) {
        controller.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0, arbFF);
    }
}
