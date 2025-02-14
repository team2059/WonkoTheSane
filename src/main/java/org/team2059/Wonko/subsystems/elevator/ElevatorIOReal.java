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
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ElevatorIOReal implements ElevatorIO {

    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private SparkClosedLoopController pidController;
    
    // Motion Profiling Components
    private ElevatorFeedforward feedforward;
    private final TrapezoidProfile.Constraints constraints;
    private final TrapezoidProfile profile;
    private TrapezoidProfile.State setpoint;
    private TrapezoidProfile.State goal;
    
    
    // Limit switches
    // private final DigitalInput[] limitSwitches = new DigitalInput[5];

    // Tunable numbers
    private LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", 0.1);
    private LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/kI", 0);
    private LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", 0);

    public ElevatorIOReal() {
        motor = new SparkMax(ElevatorConstants.motorId, MotorType.kBrushless);

        // Initialize motion profiling
        feedforward = new ElevatorFeedforward(
            ElevatorConstants.ks,   // Static Friction
            ElevatorConstants.kg,   // Gravity Compensation
            ElevatorConstants.kv);  // Velocity Feedforward

        constraints = new TrapezoidProfile.Constraints(
            ElevatorConstants.MAX_VELOCITY, 
            ElevatorConstants.MAX_ACCELERATION);

        profile = new TrapezoidProfile(constraints);
        setpoint = new TrapezoidProfile.State();
        goal = new TrapezoidProfile.State();
        
        
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
        inputs.setpointPosition = setpoint.position;
        inputs.setpointVelocity = setpoint.velocity;
        inputs.goalPosition = goal.position;

        // for (int i = 0; i < inputs.limitSwitches.length; i++) {
        //     inputs.limitSwitches[i] = limitSwitches[i].get();
        // }
    }

    @Override
    public void setGoal(double position) {
        setpoint = new TrapezoidProfile.State(encoder.getPosition(), encoder.getVelocity());
        goal = new TrapezoidProfile.State(position, 0);
    }

    @Override
    public void updateMotionProfile() {
        double prevVelocity = setpoint.velocity;
        setpoint = profile.calculate(0.02, setpoint, goal); // 20ms period
        
        // Calculate acceleration for feedforward
        double acceleration = (setpoint.velocity - prevVelocity) / 0.02;
        
        // Calculate feedforward voltage
        double ffVolts = feedforward.calculate(setpoint.velocity, acceleration);
        
        // Set position with feedforward
        pidController.setReference(
            setpoint.position,
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            ffVolts
        );
    }

    @Override
    public void hold() {
        // Apply just enough voltage to hold position against gravity
        double ffVolts = feedforward.calculate(0);
        setVoltage(ffVolts);
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