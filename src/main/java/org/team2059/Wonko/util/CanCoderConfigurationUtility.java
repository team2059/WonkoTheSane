package org.team2059.Wonko.util;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

// Methods used to configure CANcoders in real swerve drivetrain
public class CanCoderConfigurationUtility {
    public static void configureCanCoder(CANcoder canCoder) {
        // Create the new configuration
        CANcoderConfiguration config = new CANcoderConfiguration();

        // Makes the range of the sensor 0-1 so that radians can be calculated
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;

        // Makes turning ccw positive
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        // Apply cancoder configuration
        canCoder.getConfigurator().apply(config);
    }
}
