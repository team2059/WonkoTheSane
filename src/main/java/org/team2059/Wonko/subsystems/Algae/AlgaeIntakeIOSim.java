package org.team2059.Wonko.subsystems.Algae;

import org.team2059.Wonko.Constants.AlgaeIntakeConstants;
import org.team2059.Wonko.Constants.DIOConstants;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class AlgaeIntakeIOSim implements AlgaeIntakeIO {
    private final LinearSystem<N2, N1, N2> tiltPlant = 
        LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.003, 45);

    private final LinearSystem<N2, N1, N2> intakePlant = 
        LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 0.05, 3);

    
    private final DCMotorSim intakeMotorSim1 = new DCMotorSim(intakePlant, DCMotor.getNeoVortex(1), 0.003);
    private final DCMotorSim intakeMotorSim2 = new DCMotorSim(intakePlant, DCMotor.getNeoVortex(1), 0.003);
    private final DCMotorSim tiltMotorSim = new DCMotorSim(tiltPlant, DCMotor.getNEO(1), 0.0003);
    private double tiltPosition = 0.0;
    private double intakeSpeed = 0.0;
    private double tiltSpeed = 0.0;

    @Override
    public void updateInputs(AlgaeIntakeIOInputs inputs) {
        intakeMotorSim1.update(0.02);
        intakeMotorSim2.update(0.02);
        tiltMotorSim.update(0.02);

        tiltPosition += tiltSpeed * 0.02; // Simulate tilt movement
        
        inputs.intake1Current = intakeMotorSim1.getCurrentDrawAmps();
        inputs.intake1Output = intakeSpeed;
        inputs.intake1Velocity = intakeMotorSim1.getAngularVelocityRPM();
        inputs.intake1Voltage = intakeMotorSim1.getInputVoltage();

        inputs.intake2Current = intakeMotorSim2.getCurrentDrawAmps();
        inputs.intake2Output = intakeSpeed;
        inputs.intake2Velocity = intakeMotorSim2.getAngularVelocityRPM();
        inputs.intake2Voltage = intakeMotorSim2.getInputVoltage();
        


        inputs.tiltPosition = tiltPosition;
        inputs.encoderConnected = true;
    }

    @Override
    public void setIntakeSpeed(double speed) {
        this.intakeSpeed = speed;
    }

    @Override
    public void setTiltSpeed(double speed) {
        this.tiltSpeed = speed;
    }
}
