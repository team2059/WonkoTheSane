package org.team2059.Wonko.subsystems.algae;

import org.team2059.Wonko.Constants.AlgaeCollectorConstants;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class AlgaeCollectorIOReal implements AlgaeCollectorIO {
    private SparkFlex motor1;
    private SparkFlex motor2;

    private SparkMax tiltMotor;

    private DutyCycleEncoder tiltEncoder;

    // Debouncer requires a condition to occur for a certain amount of time in order
    // for the boolean to change
    // kRising: false->true
    private Debouncer debouncer = new Debouncer(0.33, DebounceType.kRising);

    public AlgaeCollectorIOReal() {
        // Create intake and tilt motors and configure them
        motor1 = new SparkFlex(AlgaeCollectorConstants.motor1Id, MotorType.kBrushless);
        motor2 = new SparkFlex(AlgaeCollectorConstants.motor2Id, MotorType.kBrushless);
        tiltMotor = new SparkMax(AlgaeCollectorConstants.tiltMotorId, MotorType.kBrushless);

        motor1.configure(
            new SparkFlexConfig()
                .inverted(false)
                .idleMode(IdleMode.kBrake), 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
        );
        motor2.configure(
            new SparkFlexConfig()
                .inverted(true)
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

        // Create thru-bore encoder and configure max/min values
        tiltEncoder = new DutyCycleEncoder(AlgaeCollectorConstants.tiltEncoderDio);
        tiltEncoder.setDutyCycleRange(AlgaeCollectorConstants.tiltEncoderMin, AlgaeCollectorConstants.tiltEncoderMax);

        debouncer.calculate(false); // Start debouncer at false
    }

    @Override
    public void setIntakeSpeed(double speed) {
        motor1.set(speed);
        motor2.set(speed);
    }

    @Override
    public void setTiltPosition(double positionRadians) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setTiltPosition'");
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

        inputs.hasAlgae = debouncer.calculate(inputs.motor1CurrentAmps > AlgaeCollectorConstants.stallDetectionAmps);

        inputs.thruBoreConnected = tiltEncoder.isConnected();
        inputs.thruBorePosition = tiltEncoder.get();
    }
}
