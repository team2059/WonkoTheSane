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

    default void updateInputs(ElevatorIOInputs inputs) {};
    default void stop() {};
    default void setVoltage(double volts) {};
    default void resetEncoder() {};
    default void setSpeed(double speed) {};
}
