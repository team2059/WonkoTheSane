package org.team2059.Wonko.util;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class SparkConfigurationUtility {
    /**
     * Configure a Spark motor controller. 
     * In 2025, REV made changes requiring use 
     * of a Spark[Max/Flex]Config object
     * 
     * @param spark The Spark to configure
     * @param inverted Boolean motor inversion value
     * @param idleMode IdleMode.kBrake or IdleMode.kCoast
     * @param positionConversionFactor MotorRotations x [This factor] = units
     * @param velocityConversionFactor MotorRotations x [This factor] = units/sec
     */
    public static void configureSpark(
        SparkFlex spark, 
        boolean inverted, 
        IdleMode idleMode,
        double positionConversionFactor,
        double velocityConversionFactor
    ) {
      SparkFlexConfig config = new SparkFlexConfig();

      config 
        .inverted(inverted)
        .idleMode(idleMode);

      config.encoder
        .positionConversionFactor(positionConversionFactor)
        .velocityConversionFactor(velocityConversionFactor);

      spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    public static void configureSpark(
        SparkMax spark, 
        boolean inverted, 
        IdleMode idleMode,
        double positionConversionFactor,
        double velocityConversionFactor
    ) {
      SparkMaxConfig config = new SparkMaxConfig();

      config 
        .inverted(inverted)
        .idleMode(idleMode);

      config.encoder
        .positionConversionFactor(positionConversionFactor)
        .velocityConversionFactor(velocityConversionFactor);

      spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
