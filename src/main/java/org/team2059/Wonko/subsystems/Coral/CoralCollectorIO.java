package org.team2059.Wonko.subsystems.Coral;

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

        public boolean thruBoreConnected = false;
        public double thruBorePos = 0.0;

        public boolean hasCoral = false;
    }

    public void updateInputs(CoralCollectorIOInputs inputs);

    public void setIntakeSpeed(double speed);

    public void setTiltPosition(double position);

    public void stopIntake();

    public void stopTilt();

    public void stopAll();
}