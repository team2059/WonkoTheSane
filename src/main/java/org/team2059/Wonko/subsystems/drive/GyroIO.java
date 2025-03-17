package org.team2059.Wonko.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean connected = false;
        public double yaw = 0;
    }

    default public void updateInputs(GyroIOInputs inputs) {};

    default public void reset() {};
}
