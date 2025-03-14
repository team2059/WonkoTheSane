package org.team2059.Wonko.subsystems.coral;

import static edu.wpi.first.units.Units.*;

import org.team2059.Wonko.Constants.CoralCollectorConstants;
import org.team2059.Wonko.util.LoggedTunableNumber;

import com.revrobotics.AbsoluteEncoder;
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
import edu.wpi.first.wpilibj.DigitalInput;

public class CoralCollectorIOReal implements CoralCollectorIO {
    // Our motors...
    private SparkFlex intakeMotor;
    private SparkFlex tiltMotor;

    // PID controllers (these are onboard the Sparks, run at 10ms (min. 2x as fast as Rio))
    private SparkClosedLoopController flywheelController;
    private SparkClosedLoopController tiltController;

    private AbsoluteEncoder tiltAbsoluteEnc;
    
    // IR sensor - detects whether coral is present by light signal on or off
    private DigitalInput irSensor;

    // Tunable numbers
    private LoggedTunableNumber kPIntake = new LoggedTunableNumber("CoralCollector/Intake/kP", CoralCollectorConstants.kPIntake); // velocity control PID vals are very low, this is normal.
    private LoggedTunableNumber kFIntake = new LoggedTunableNumber("CoralCollector/Intake/kF", CoralCollectorConstants.kFIntake); // Feedforward constant.. not sure units
    private LoggedTunableNumber kPTilt = new LoggedTunableNumber("CoralCollector/Tilt/kP", CoralCollectorConstants.kPTilt);
    private LoggedTunableNumber kITilt = new LoggedTunableNumber("CoralCollector/Tilt/kI", CoralCollectorConstants.kITilt);
    private LoggedTunableNumber kDTilt = new LoggedTunableNumber("CoralCollector/Tilt/kD", CoralCollectorConstants.kDTilt);

    public CoralCollectorIOReal() {
        intakeMotor = new SparkFlex(CoralCollectorConstants.intakeMotorId, MotorType.kBrushless);
        tiltMotor = new SparkFlex(CoralCollectorConstants.tiltMotorId, MotorType.kBrushless);

        // Configure both Spark motor controllers
        SparkFlexConfig intakeConfig = new SparkFlexConfig();
        intakeConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake);
        intakeConfig.closedLoop
            .pidf(kPIntake.get(), 0, 0, kFIntake.get())
            .outputRange(-1, 1);
        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        flywheelController = intakeMotor.getClosedLoopController(); 
        intakeMotor.clearFaults(); 

        SparkFlexConfig tiltConfig = new SparkFlexConfig();
        tiltConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(((int)CoralCollectorConstants.tiltCurrentLimit.in(Amps)))
            .closedLoopRampRate(0.5);
        tiltConfig.absoluteEncoder
            .zeroOffset(0.638)
            .inverted(true)
            .positionConversionFactor(2.0 * Math.PI)
            .velocityConversionFactor(2.0 * Math.PI / 60)
            .zeroCentered(true);
        tiltConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(kPTilt.get(), kITilt.get(), kDTilt.get())
            .outputRange(-1, 1);
        tiltMotor.configure(tiltConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        tiltController = tiltMotor.getClosedLoopController(); 
        tiltMotor.clearFaults(); 

        tiltAbsoluteEnc = tiltMotor.getAbsoluteEncoder();

        // Configure IR sensor - reports presence of coral
        irSensor = new DigitalInput(CoralCollectorConstants.irSensorDio);
    }

    @Override
    public void updateInputs(CoralCollectorIOInputs inputs) {

        // Update tunables
        LoggedTunableNumber.ifChanged(
            hashCode(), 
            () -> {
                SparkFlexConfig tempIntakeConfig = new SparkFlexConfig();
                tempIntakeConfig.closedLoop.pidf(kPIntake.get(), 0, 0, kFIntake.get());
                intakeMotor.configure(tempIntakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

                SparkFlexConfig tempTiltConfig = new SparkFlexConfig();
                tempTiltConfig.closedLoop.pid(kPTilt.get(), kITilt.get(), kDTilt.get());
                tiltMotor.configure(tempTiltConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            }, 
            kPIntake, kFIntake, kPTilt, kITilt, kDTilt
        );

        inputs.intakeMotorAppliedVolts = (intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage());
        inputs.tiltMotorAppliedVolts = (tiltMotor.getAppliedOutput() * tiltMotor.getBusVoltage());
        
        inputs.intakeMotorCurrentAmps = intakeMotor.getOutputCurrent();
        inputs.tiltMotorCurrentAmps = tiltMotor.getOutputCurrent();

        inputs.intakeMotorTemp = intakeMotor.getMotorTemperature();
        inputs.tiltMotorTemp = tiltMotor.getMotorTemperature();

        inputs.tiltAbsPosRadians = tiltAbsoluteEnc.getPosition();
        inputs.tiltMotorVelocityRadPerSec = tiltAbsoluteEnc.getVelocity();

        inputs.hasCoral = !irSensor.get();

        inputs.intakeMotorSpeed = intakeMotor.getEncoder().getVelocity();
    }

    @Override
    public void setIntakeSpeed(double speed) {
        flywheelController.setReference(
            speed, 
            ControlType.kVelocity, 
            ClosedLoopSlot.kSlot0
        );
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
