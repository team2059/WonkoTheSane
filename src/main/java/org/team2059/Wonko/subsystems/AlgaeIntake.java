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
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntake extends SubsystemBase {
  private final SparkFlex motor1;
  private final SparkFlex motor2;

  public DutyCycleEncoder algaeTiltThruBoreEncoder;

  /** Creates a new algaeEndEffector. */
  public AlgaeIntake() {
    motor1 = new SparkFlex(AlgaeIntakeConstants.motor1ID, MotorType.kBrushless);
    motor2 = new SparkFlex(AlgaeIntakeConstants.motor2ID, MotorType.kBrushless);

    SparkFlexConfig config1 = new SparkFlexConfig();
    config1.inverted(false);
    config1.idleMode(IdleMode.kBrake);
    motor1.configure(config1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkFlexConfig config2 = new SparkFlexConfig();
    config2.inverted(true); // Motor2 opposite direction
    config2.idleMode(IdleMode.kBrake);
    motor2.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    algaeTiltThruBoreEncoder = new DutyCycleEncoder(DIOConstants.kAlgaeTiltThruBoreEncoderDIO);

    algaeTiltThruBoreEncoder.setDutyCycleRange(0.02, 0.98); // EXAMPLE RANGE CHANGE LATER
    algaeTiltThruBoreEncoder.setInverted(true);
    algaeTiltThruBoreEncoder.setAssumedFrequency(1000); // Example frequency

  }

  public void setEndEffectorSpeed(double speed) {
    // This needs to be changed
    motor1.set(speed);
    motor2.set(speed);
  }

  public void holdAlgae(double speed) {
    motor1.set(speed);
    motor2.set(speed);
  }

  public SparkFlex getMotor2() {
    return motor2;
  }

  public SparkFlex getMotor1() {
    return motor1;
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
    Logger.recordOutput("Current", motor1.getOutputCurrent());
    Logger.recordOutput("id25 output", motor1.getAppliedOutput());

    Logger.recordOutput("AlgaeIntake/TiltPosition", getAbsolutePosition());
  }
}
