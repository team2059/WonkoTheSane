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
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntake extends SubsystemBase {
  private final SparkMax intakeMotor;
  private final SparkFlex tiltMotor;
  public DutyCycleEncoder coralTiltThruBoreEncoder;

  /** Creates a new CoralIntake. */
  public CoralIntake() {
    intakeMotor = new SparkMax(CoralIntakeConstants.intakeMotorID, MotorType.kBrushless);
    tiltMotor = new SparkFlex(CoralIntakeConstants.tiltMotorID, MotorType.kBrushless);

    // configure intake spark
    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(false);
    config.idleMode(IdleMode.kBrake);
    intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // configure tilt spark
    SparkFlexConfig tiltConfig = new SparkFlexConfig();
    tiltConfig.inverted(false);
    tiltConfig.idleMode(IdleMode.kBrake);
    tiltMotor.configure(tiltConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    coralTiltThruBoreEncoder = new DutyCycleEncoder(DIOConstants.kCoralTiltThruBoreEncoderDIO);

    coralTiltThruBoreEncoder.setDutyCycleRange(0.02, 0.98); // EXAMPLE RANGE CHANGE LATER
    coralTiltThruBoreEncoder.setInverted(false);


  }

  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public void setTiltSpeed(double speed) {
    tiltMotor.set(speed);
  }

  public SparkMax getIntakeMotor() {
    return intakeMotor;
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

  @Override
  public void periodic() {
    Logger.recordOutput("Coral/AbsoluteEncoderPosition", coralTiltThruBoreEncoder.get());
    // This method will be called once per scheduler run
  }
}
