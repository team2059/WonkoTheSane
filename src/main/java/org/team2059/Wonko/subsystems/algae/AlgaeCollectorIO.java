package org.team2059.Wonko.subsystems.algae;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeCollectorIO {
    @AutoLog
    class AlgaeCollectorIOInputs {
        public double motor1AppliedVolts = 0.0;
        public double motor2AppliedVolts = 0.0;
        public double tiltMotorAppliedVolts = 0.0;
        
        public double motor1CurrentAmps = 0.0;
        public double motor2CurrentAmps = 0.0;
        public double tiltMotorCurrentAmps = 0.0;

        public double motor1Temp = 0.0;
        public double motor2Temp = 0.0;
        public double tiltMotorTemp = 0.0;

        public boolean hasAlgae;

        public boolean thruBoreConnected = false;
        public double thruBorePositionDegrees = 0.0;
    }

    default void updateInputs(AlgaeCollectorIOInputs inputs) {};

    default void setIntakeSpeed(double speed) {};

    default void setTiltSpeed(double speed) {};

    default void stopIntake() {};

    default void stopTilt() {};

    default void stopAll() {};
}
