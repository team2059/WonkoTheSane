package org.team2059.Wonko.subsystems.coral;

import org.littletonrobotics.junction.AutoLog;

public interface CoralCollectorIO {
    @AutoLog
    class CoralCollectorIOInputs {
        public double intakeMotorAppliedVolts = 0.0;
        public double tiltMotorAppliedVolts = 0.0;

        public double intakeMotorCurrentAmps = 0.0;
        public double tiltMotorCurrentAmps = 0.0;

        public double intakeMotorTemp = 0.0;
        public double tiltMotorTemp = 0.0;

        public double tiltAbsPosRadians = 0.0;
        public double tiltMotorVelocityRadPerSec = 0.0;

        public boolean hasCoral = false;

        public double intakeMotorSpeed = 0.0;
    }

    /**
     * @param inputs to be updated (pass by reference, of course)
     */
    default void updateInputs(CoralCollectorIOInputs inputs) {};

    /**
     * Set speed of intake flywheel.
     * @param speed in RPM
     */
    default void setIntakeSpeed(double speed) {};

    /**
     * Set speed of tilt motor.
     * @param speed value [-1, 1]
     */
    default void setTiltSpeed(double speed) {};

    /**
     * Set voltage of the tilt motor. [CAREFUL...]
     * @param volts within [-12, 12]
     */
    default void setTiltVolts(double volts) {};

    /**
     * Immediately stop intake by setting 0 volts.
     */
    default void stopIntake() {};

    /**
     * Immediately stop tilt by setting 0 volts.
     */
    default void stopTilt() {};

    /**
     * Immediately stop intake & tilt by setting 0 volts.
     */
    default void stopAll() {};

    /**
     * Set the PID setpoint for onboard Spark PID controller.
     * @param posRadians setpoint position, in radians
     */
    default void setTiltPos(double posRadians) {};

}