package org.team2059.Wonko.subsystems.Algae;

import org.team2059.Wonko.Constants.AlgaeIntakeConstants;
import org.team2059.Wonko.Constants.DIOConstants;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

public class AlgaeIntakeIOSim implements AlgaeIntakeIO {
    private final LinearSystem<N2, N1, N2> tiltPlant = 
        LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.003, 45);
    
        
    private final LinearSystem<N2, N1, N2> intakePlant = 
        LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 0.05, 3);

    private final DCMotorSim intakeMotorSim1 = new DCMotorSim(intakePlant, DCMotor.getNeoVortex(2), 0.003);
    private final DCMotorSim intakeMotorSim2 = new DCMotorSim(intakePlant, DCMotor.getNeoVortex(2), 0.003);

    private final DCMotorSim tiltMotorSim = new DCMotorSim(tiltPlant, DCMotor.getNEO(1), 0.0003);

    private double tiltPosition = 0.0;
    private double intakeSpeed = 0.0;

    private boolean isGamepieceDetected = false;

    private final PIDController tiltPidController = new PIDController(AlgaeIntakeConstants.kPAlgae, AlgaeIntakeConstants.kIAlgae, AlgaeIntakeConstants.kDAlgae);

    @Override
    public void updateInputs(AlgaeIntakeIOInputs inputs) {
        intakeMotorSim1.update(0.02);
        intakeMotorSim2.update(0.02);
        tiltMotorSim.update(0.02);

        
        inputs.intake1Current = intakeMotorSim1.getCurrentDrawAmps();
        inputs.intake1Output = intakeSpeed;
        inputs.intake1Velocity = intakeMotorSim1.getAngularVelocityRPM();
        inputs.intake1Voltage = intakeMotorSim1.getInputVoltage();

        
        inputs.intake2Current = intakeMotorSim1.getCurrentDrawAmps();
        inputs.intake2Output = intakeSpeed;
        inputs.intake2Velocity = intakeMotorSim1.getAngularVelocityRPM();
        inputs.intake2Voltage = intakeMotorSim1.getInputVoltage();
        
        inputs.tiltPosition = tiltPosition;
        isGamepieceDetected = inputs.intake1Current > AlgaeIntakeConstants.INTAKE_STALL_DETECTION;
        inputs.hasAlgae = isAlgae();
    }

    @Override
    public void setIntakeSpeed(double speed) {
        intakeMotorSim1.setInputVoltage(MathUtil.clamp(12 * speed, -12, 12));
        intakeMotorSim2.setInputVoltage(MathUtil.clamp(12 * speed, -12, 12));
    }

    @Override
    public void setTiltPosition(double positionRadians) {
        double output = tiltPidController.calculate(tiltMotorSim.getAngularPositionRad(), positionRadians); 
        setTiltVoltage(output);
    }

    @Override
    public boolean isAlgae() {
        return isGamepieceDetected; 
    } 
    
    @Override
    public void setTiltVoltage(double voltage) {
        tiltMotorSim.setInput(MathUtil.clamp(voltage, -12, 12));
    }

    public void stopTilt() {
        tiltMotorSim.setInputVoltage(0);
    }

    public void stopIntake() {
        intakeMotorSim1.setInputVoltage(0);
        intakeMotorSim2.setInputVoltage(0);
    }

    public void stopAll() {
        stopTilt();
        stopIntake();
    }
}
