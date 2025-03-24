package org.team2059.Wonko.subsystems.algae;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeCollectorIO {
    @AutoLog
    class AlgaeCollectorIOInputs {
        public double intakeAppliedVolts = 0.0;
        public double tiltMotorAppliedVolts = 0.0;
        
        public double intakeCurrentAmps = 0.0;
        public double tiltMotorCurrentAmps = 0.0;

        public double intakeTemp = 0.0;
        public double tiltMotorTemp = 0.0;

        public double tiltAbsPosRadians = 0.0;
        public double tiltMotorVelocityRadPerSec = 0.0;

        public boolean hasAlgae;
    }

    default void updateInputs(AlgaeCollectorIOInputs inputs) {};

    default void setIntakeSpeed(double speed) {};

    default void setTiltSpeed(double speed) {};

    default void setTiltVolts(double volts) {};

    default void stopIntake() {};

    default void stopTilt() {};

    default void stopAll() {};

    default void setTiltPos(double posRadians) {};
}
