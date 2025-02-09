package org.team2059.Wonko.subsystems.Algae;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeIntakeIO {
    @AutoLog
    class AlgaeIntakeIOInputs {

        public double intake1Current = 0.0;  // Current drawn by intake motor 1
        public double intake1Output = 0.0;  // Output percentage 
        public double intake1Voltage = 0.0; // Applied voltage
        public double intake1Velocity = 0.0; // RPM of intake motor 1

        public double intake2Current = 0.0;  // Current drawn by intake motor 2
        public double intake2Output = 0.0;  // Output percentage
        public double intake2Voltage = 0.0; // Applied voltage
        public double intake2Velocity = 0.0; // RPM of intake motor 2

        public double tiltPosition = 0.0;  // Position of tilt throughbore
        public double tiltVelocity = 0.0;  // Velocity of tilt
        public boolean hasAlgae = false;  // if encoder detects (TODO: Get encoder in SIM)
        public boolean encoderConnected = false; // Encoder connection status
    }

    public void updateInputs(AlgaeIntakeIOInputs inputs);

    public default void setIntakeSpeed(double speed) {}
    public default void setTiltSpeed(double speed) {}
}

