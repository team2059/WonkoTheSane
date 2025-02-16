package org.team2059.Wonko.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    class ClimberIOInputs {
        public double climbMotor1AppliedVolts = 0.0;
        public double climbMotor2AppliedVolts = 0.0;

        public double climbMotor1CurrentAmps = 0.0;
        public double climbMotor2CurrentAmps = 0.0;

        public double climbMotor1Temp = 0.0;
        public double climbMotor2Temp = 0.0; 

    }

    default void updateInputs(ClimberIOInputs inputs) {};

    default void setClimbSpeed(double speed) {};  

    default void stopClimb() {};
}
