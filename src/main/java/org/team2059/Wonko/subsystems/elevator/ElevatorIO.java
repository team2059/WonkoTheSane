package org.team2059.Wonko.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    
    @AutoLog
    class ElevatorIOInputs {
        public double positionMeters = 0.0;
        public double velocityMetersPerSecond = 0.0;

        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;

        public double motorTemp = 0.0;

        // public boolean[] limitSwitches = new boolean[5];
    }

    public void updateInputs(ElevatorIOInputs inputs);

    public void stop();

    public void setVoltage(double volts);

    public void resetEncoder();

    public void setLevel(int level);

    public void setPosition(double positionMeters);

    public void setVelocity(double velocityMps);
    
}
