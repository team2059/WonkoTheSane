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
        public double tiltMotorPos = 0.0;

        public boolean thruBoreConnected = false;
        public double thruBorePositionDegrees = 0.0;

        public boolean hasCoral = false;
    }

    default void updateInputs(CoralCollectorIOInputs inputs) {};

    default void setIntakeSpeed(double speed) {};

    default void setTiltSpeed(double speed) {};

    default void setTiltVolts(double volts) {};

    default void stopIntake() {};

    default void stopTilt() {};

    default void stopAll() {};
}