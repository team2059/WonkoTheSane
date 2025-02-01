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
  public DutyCycleEncoder coralTiltThruBoreEncoder;
  public DigitalInput irSensor;

  /** Creates a new CoralIntake. */
  public CoralIntake() {
    intakeMotor = new SparkFlex(CoralIntakeConstants.intakeMotorID, MotorType.kBrushless);
    tiltMotor = new SparkMax(CoralIntakeConstants.tiltMotorID, MotorType.kBrushless);

    // configure spark
    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(false);
    config.idleMode(IdleMode.kBrake);
    intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    coralTiltThruBoreEncoder = new DutyCycleEncoder(DIOConstants.kCoralTiltThruBoreEncoderDIO);

    coralTiltThruBoreEncoder.setDutyCycleRange(0.02, 0.98); // TODO: EXAMPLE RANGE CHANGE LATER
    coralTiltThruBoreEncoder.setInverted(true); // TODO: SEE IF INVERTED

    irSensor = new DigitalInput(CoralIntakeConstants.irSensorDIO);
  }

  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public SparkFlex getIntakeMotor() {
    return intakeMotor;
  }

  public void setTiltSpeed(double speed) {
    tiltMotor.set(speed);
  }

  public SparkMax getTiltMotor() {
    return tiltMotor;
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
    return irSensor.get(); // TODO: MIGHT HAVE TO INVERT 
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Throughbore position", getAbsolutePosition());
    Logger.recordOutput("Has Coral?", isCoralPresent());
  }
}
