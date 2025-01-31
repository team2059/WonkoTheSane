// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2059.Wonko.subsystems;

import org.littletonrobotics.junction.Logger;
import org.team2059.Wonko.Constants.AlgaeIntakeConstants;
import org.team2059.Wonko.Constants.CoralIntakeConstants;
import org.team2059.Wonko.Constants.DIOConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntake extends SubsystemBase {
  private final SparkMax motor;
  public DutyCycleEncoder coralTiltThruBoreEncoder;
  public DigitalInput irSensor;

  /** Creates a new CoralIntake. */
  public CoralIntake() {
    motor = new SparkMax(CoralIntakeConstants.intakeMotorID, MotorType.kBrushless);

    // configure spark
    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(false);
    config.idleMode(IdleMode.kBrake);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    coralTiltThruBoreEncoder = new DutyCycleEncoder(DIOConstants.kCoralTiltThruBoreEncoderDIO);

    coralTiltThruBoreEncoder.setDutyCycleRange(0.02, 0.98); // EXAMPLE RANGE CHANGE LATER
    coralTiltThruBoreEncoder.setInverted(true);

    irSensor = new DigitalInput(CoralIntakeConstants.irSensorDIO);
  }

  public void setIntakeSpeed(double speed) {
    motor.set(speed);
  }

  public SparkMax getMotor() {
    return motor;
  }

  public double getAbsolutePosition() {
    return coralTiltThruBoreEncoder.get() - CoralIntakeConstants.throughBoreOffset;
  }

  public double getRawEncoderPosition() {
    return coralTiltThruBoreEncoder.get();
  }

  public boolean isEncoderConnected() {
    return coralTiltThruBoreEncoder.isConnected();
  }

  public boolean isCoralPresent() {
    return irSensor.get();
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Throughbore position", getAbsolutePosition());
  }
}
