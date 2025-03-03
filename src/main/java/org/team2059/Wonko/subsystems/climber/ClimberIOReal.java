package org.team2059.Wonko.subsystems.climber;

import org.team2059.Wonko.Constants.ClimberConstants;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ClimberIOReal implements ClimberIO {
    private SparkMax motor1; 
    private SparkMax motor2; 

    private DutyCycleEncoder climberThroughbore;
    
        public ClimberIOReal() {
            motor1 = new SparkMax(ClimberConstants.motor1ID, MotorType.kBrushless);
            motor2 = new SparkMax(ClimberConstants.motor2ID, MotorType.kBrushless);
    
            SparkMaxConfig motor1Config = new SparkMaxConfig();
            motor1Config    
                .inverted(false)
                .idleMode(IdleMode.kBrake);
            motor1.configure(motor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            motor1.clearFaults();
    
            SparkMaxConfig motor2Config = new SparkMaxConfig();
            motor2Config    
                .inverted(true)
                .idleMode(IdleMode.kBrake);
            motor2.configure(motor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            motor2.clearFaults(); 
    
            climberThroughbore = new DutyCycleEncoder(ClimberConstants.climbThroughBoreDIO);
        }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.climbMotor1AppliedVolts = motor1.getAppliedOutput() * motor1.getBusVoltage();
        inputs.climbMotor2AppliedVolts = motor2.getAppliedOutput() * motor2.getBusVoltage();

        inputs.climbMotor1CurrentAmps = motor1.getOutputCurrent();
        inputs.climbMotor2CurrentAmps = motor2.getOutputCurrent();

        inputs.climbMotor1Temp = motor1.getMotorTemperature(); 
        inputs.climbMotor2Temp = motor2.getMotorTemperature();

        inputs.tiltThroughboreConnected = climberThroughbore.isConnected();
        inputs.tiltThroughborePosition = climberThroughbore.get() * 2.0 * Math.PI;
    }

    @Override
    public void setClimbSpeed(double speed) {
        motor1.set(speed);
        motor2.set(speed);
    } 

    @Override
    public void stopClimb() {
        motor1.set(0);
        motor2.set(0);
    }

    public void setVoltage(double volts) {
        motor1.set(MathUtil.clamp(volts, -12, 12));
        motor2.set(MathUtil.clamp(volts, -12, 12));
    }
    }

