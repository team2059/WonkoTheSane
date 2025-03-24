package org.team2059.Wonko.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    
    @AutoLog
    class ElevatorIOInputs {
        public double positionMeters = 0.0;
        public double velocityMetersPerSecond = 0.0;

        public double rightMotorAppliedVolts = 0.0;
        public double rightMotorCurrentAmps = 0.0;

        public double rightMotorTemp = 0.0;

        public double leftMotorAppliedVolts = 0.0;
        public double leftMotorCurrentAmps = 0.0;

        public double leftMotorTemp = 0.0;

        public boolean zeroLimit = false;
    }

    /**
     * @param inputs to be updated (pass by reference, of course)
     */
    default void updateInputs(ElevatorIOInputs inputs) {};

    /**
     * Stop the motor immediately by setting 0 volts
     */
    default void stop() {};
    
    /**
     * Set the voltage of the motor [-12, 12] volts (be careful!)
     * @param volts
     */
    default void setVoltage(double volts) {};
    
    /**
     * Sets the built-in encoder reading to 0
     */
    default void resetEncoder() {};
    
    /**
     * Set the speed of the motor, [-1, 1]
     * @param speed
     */
    default void setSpeed(double speed) {};
    
    /**
     * Set the setpoint position for built-in Spark PID controller
     * @param position target position in meters
     * @param arbFF arbitrary feedforward value in volts
     */
    default void setPositionClosedLoopWithFF(double position, double arbFF) {};
}
