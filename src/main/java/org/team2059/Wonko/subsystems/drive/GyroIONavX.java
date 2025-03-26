package org.team2059.Wonko.subsystems.drive;

import com.studica.frc.AHRS;

public class GyroIONavX implements GyroIO{

    private final AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

    private boolean is180Rotated = false;

    public GyroIONavX() {
        reset();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = gyro.isConnected();
        inputs.is180Rotated = is180Rotated;

        if (inputs.is180Rotated) {
            inputs.yaw = gyro.getYaw() + 180;
        } else {
            inputs.yaw = gyro.getYaw();
        }
    }

    @Override
    public void reset() {
        gyro.reset();
    }

    @Override
    public void set180Rotation(boolean enabled) {
        is180Rotated = enabled;
    }
    
}
