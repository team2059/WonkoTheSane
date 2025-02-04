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
  private final SparkFlex intakeMotor;
  private final SparkMax tiltMotor; 
  // Throughbore encoder is a DutyCycleEncoder, goes from 0-1
  public DutyCycleEncoder coralTiltThruBoreEncoder;
  // IR sensor is a digital sensor that detects if light is reflected (returns 0 or 1)
  public DigitalInput irSensor;

  /** Creates a new CoralIntake. */
  public CoralIntake() {
    // Creating tilt and intake motor
    intakeMotor = new SparkFlex(CoralIntakeConstants.intakeMotorID, MotorType.kBrushless);
    tiltMotor = new SparkMax(CoralIntakeConstants.tiltMotorID, MotorType.kBrushless);

    // configure spark
    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(false);
    config.idleMode(IdleMode.kBrake);
    intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Creating coral throughbore
    coralTiltThruBoreEncoder = new DutyCycleEncoder(DIOConstants.kCoralTiltThruBoreEncoderDIO);

    // Adding range
    coralTiltThruBoreEncoder.setDutyCycleRange(0.02, 0.98); // TODO: EXAMPLE RANGE CHANGE LATER
    // Might be inverted 
    coralTiltThruBoreEncoder.setInverted(true); // TODO: SEE IF INVERTED

    irSensor = new DigitalInput(CoralIntakeConstants.irSensorDIO);
  }

  // Setting intake speed using params given
  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

  // Getting intake motor
  public SparkFlex getIntakeMotor() {
    return intakeMotor;
  }
  
  // Setting tilt speed using params given
  public void setTiltSpeed(double speed) {
    tiltMotor.set(speed);
  }

  // Getting tilt motor
  public SparkMax getTiltMotor() {
    return tiltMotor;
  }

  // Getting throughbore position by subtracting offset 
  public double getAbsolutePosition() {
    return coralTiltThruBoreEncoder.get() - CoralIntakeConstants.throughBoreOffset;
  }

  // Getting true throughbore position
  public double getRawEncoderPosition() {
    return coralTiltThruBoreEncoder.get();
  }

  // Sees if encoder is connected 
  public boolean isEncoderConnected() {
    return coralTiltThruBoreEncoder.isConnected();
  }

  // Checks if coral is present by returning value from ir sensor (will be 0 or 1)
  public boolean isCoralPresent() {
    return irSensor.get(); // TODO: MIGHT HAVE TO INVERT 
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Throughbore position", getAbsolutePosition());
    Logger.recordOutput("Has Coral?", isCoralPresent());
  }
}
