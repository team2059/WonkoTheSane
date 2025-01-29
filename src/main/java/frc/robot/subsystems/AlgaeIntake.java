// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeIntakeConstants;

public class AlgaeIntake extends SubsystemBase {
  private final SparkMax motor1;
  private final SparkMax motor2;

  /** Creates a new algaeEndEffector. */
  public AlgaeIntake() {
    motor1 = new SparkMax(AlgaeIntakeConstants.motor1ID, MotorType.kBrushless);
    motor2 = new SparkMax(AlgaeIntakeConstants.motor2ID, MotorType.kBrushless);

    SparkMaxConfig config1 = new SparkMaxConfig();
    config1.inverted(false);
    config1.idleMode(IdleMode.kBrake);
    motor1.configure(config1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig config2 = new SparkMaxConfig();
    config2.inverted(true); // Motor2 opposite direction
    config2.idleMode(IdleMode.kBrake);
    motor2.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setEndEffectorSpeed(double speed) {
    // This needs to be changed
    motor1.set(speed);
    motor2.set(-speed);
  }

  public void setHoldSpeed(double speed) {
    // This needs to be changed
    motor1.set(speed);
    motor2.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
