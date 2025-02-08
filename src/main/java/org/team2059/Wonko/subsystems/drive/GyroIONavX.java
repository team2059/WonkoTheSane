package org.team2059.Wonko.subsystems.drive;

import com.studica.frc.AHRS;

public class GyroIONavX implements GyroIO{


    private final AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

    public GyroIONavX() {
        reset();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = gyro.isConnected();
        inputs.yaw = gyro.getYaw();
    }

    @Override
    public void reset() {
        gyro.reset();
    }
    
}
