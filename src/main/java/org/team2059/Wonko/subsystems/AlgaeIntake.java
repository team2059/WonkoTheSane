// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Fuck you Mr. Potter
package org.team2059.Wonko.subsystems;

import org.littletonrobotics.junction.Logger;
import org.team2059.Wonko.Constants.AlgaeIntakeConstants;
import org.team2059.Wonko.Constants.DIOConstants;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntake extends SubsystemBase {
  private final SparkFlex intakeMotor1;
  private final SparkFlex intakeMotor2;
  private final SparkMax tiltMotor; 
  public boolean hasAlgae = false; 

  public DutyCycleEncoder algaeTiltThruBoreEncoder;

  /** Creates a new algaeEndEffector. */
  public AlgaeIntake() {
    intakeMotor1 = new SparkFlex(AlgaeIntakeConstants.intakeMotor1ID, MotorType.kBrushless);
    intakeMotor2 = new SparkFlex(AlgaeIntakeConstants.intakeMotor2ID, MotorType.kBrushless);
    tiltMotor = new SparkMax(AlgaeIntakeConstants.tiltMotorID, MotorType.kBrushless);


    SparkFlexConfig config1 = new SparkFlexConfig();
    config1.inverted(false);
    config1.idleMode(IdleMode.kBrake);
    intakeMotor1.configure(config1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkFlexConfig config2 = new SparkFlexConfig();
    config2.inverted(true); // intakeMotor2 opposite direction
    config2.idleMode(IdleMode.kBrake);
    intakeMotor2.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    algaeTiltThruBoreEncoder = new DutyCycleEncoder(DIOConstants.kAlgaeTiltThruBoreEncoderDIO);

    algaeTiltThruBoreEncoder.setDutyCycleRange(0.02, 0.98); // TODO: EXAMPLE RANGE CHANGE LATER
    algaeTiltThruBoreEncoder.setInverted(true); // TODO: SEE IF INVERTED

  }

  public void setEndEffectorSpeed(double speed) {
    // This needs to be changed
    intakeMotor1.set(speed);
    intakeMotor2.set(speed);
  }

  public void setTiltSpeed(double speed) {
    tiltMotor.set(speed);
  }

  public SparkFlex getMotor2() {
    return intakeMotor2;
  }

  public SparkFlex getMotor1() {
    return intakeMotor1;
  }
  
  public SparkMax getTiltMotor() {
    return tiltMotor;
  }

  public double getAbsolutePosition() {
    return algaeTiltThruBoreEncoder.get() - AlgaeIntakeConstants.throughBoreOffset;
  }

  public double getRawEncoderPosition() {
    return algaeTiltThruBoreEncoder.get();
  }

  public boolean isEncoderConnected() {
    return algaeTiltThruBoreEncoder.isConnected();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Current", intakeMotor1.getOutputCurrent());
    Logger.recordOutput("id25 output", intakeMotor1.getAppliedOutput());

    Logger.recordOutput("AlgaeIntake/TiltPosition", getAbsolutePosition());
    Logger.recordOutput("Has algae", hasAlgae);
  }
}
