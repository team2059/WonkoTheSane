package org.team2059.Wonko.subsystems.Coral;

import org.team2059.Wonko.Constants.CoralIntakeConstants;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class CoralCollectorIOSim implements CoralCollectorIO {
    private final LinearSystem<N2, N1, N2> intakePlant;
    private final LinearSystem<N2, N1, N2> tiltPlant; 

    private final DCMotorSim intakeMotorSim; 
    private final DCMotorSim tiltMotorSim;

    private final PIDController tiltPidController; 

    public CoralCollectorIOSim() {
    intakePlant = 
        LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.003, 3);
    
    tiltPlant = 
            LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 0.05, 9);

    intakeMotorSim = new DCMotorSim(intakePlant, DCMotor.getNeoVortex(2), 0.003);
    tiltMotorSim = new DCMotorSim(tiltPlant, DCMotor.getNEO(1), 0.0003);

    tiltPidController = new PIDController(CoralIntakeConstants.kPCoral, CoralIntakeConstants.kICoral, CoralIntakeConstants.kDCoral);
    }

    @Override
    public void updateInputs(CoralCollectorIOInputs inputs) {
        inputs.intakeMotorAppliedVolts = (intakeMotorSim.getInputVoltage());
        inputs.tiltMotorAppliedVolts = (tiltMotorSim.getInputVoltage());
        inputs.intakeMotorCurrentAmps = intakeMotorSim.getCurrentDrawAmps();
        inputs.tiltMotorCurrentAmps = tiltMotorSim.getCurrentDrawAmps();
        inputs.thruBorePos = tiltMotorSim.getAngularPositionRad();
    }

    @Override
    public void setIntakeSpeed(double speed) {
        intakeMotorSim.setInputVoltage(MathUtil.clamp(speed * 12, -12, 12));
    }

    @Override
    public void setTiltPosition(double positionRadians) {
        double output = tiltPidController.calculate(tiltMotorSim.getAngularPositionRad(), positionRadians); 
        tiltMotorSim.setInputVoltage(MathUtil.clamp(output, -12, 12));
    }

    @Override
    public void stopIntake() {
        intakeMotorSim.setInputVoltage(0);
    }

    @Override
    public void stopTilt() {
        tiltMotorSim.setInputVoltage(0);
    }

    @Override
    public void stopAll() {
        stopTilt();
        stopIntake();
    }
}