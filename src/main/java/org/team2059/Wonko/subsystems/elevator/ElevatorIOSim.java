package org.team2059.Wonko.subsystems.elevator;

// import static edu.wpi.first.units.Units.*;

import org.team2059.Wonko.Constants.ElevatorConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ElevatorIOSim implements ElevatorIO {
    private DCMotorSim sim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.025, 9), 
        DCMotor.getNEO(1)
    );

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        sim.update(0.02);

        inputs.positionMeters = sim.getAngularPositionRotations() * ElevatorConstants.positionConversionFactor;
        inputs.velocityMetersPerSecond = sim.getAngularVelocityRPM() * ElevatorConstants.velocityConversionFactor;

        inputs.appliedVolts = sim.getInputVoltage();
        inputs.currentAmps = sim.getCurrentDrawAmps();
    }

    @Override
    public void stop() {
        sim.setInputVoltage(0);
    }

    @Override
    public void setVoltage(double volts) {
        sim.setInputVoltage(MathUtil.clamp(volts, -12, 12));
    }
}
