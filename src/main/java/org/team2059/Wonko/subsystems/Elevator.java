// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2059.Wonko.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.team2059.Wonko.Constants;
import org.team2059.Wonko.Constants.ElevatorConstants;
import org.team2059.Wonko.routines.ElevatorRoutine;

public class Elevator extends SubsystemBase {
  public final SparkFlex elevatorMotor;
  private final ProfiledPIDController profiledPIDController;
  private final ElevatorFeedforward feedforward;
  private final TrapezoidProfile.Constraints constraints;
  private final DigitalInput levelOne, levelTwo, levelThree, levelFour;
  public final ElevatorRoutine elevatorRoutine;

  private double setpoint = 0.0;

  /** Creates a new Elevator. */
  public Elevator() {
    elevatorMotor = new SparkFlex(Constants.ElevatorConstants.elevatorMotorID, MotorType.kBrushless);

    SparkFlexConfig config = new SparkFlexConfig();
    config.inverted(true);
    config.idleMode(IdleMode.kBrake);
    elevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Motion constraints
    constraints = new TrapezoidProfile.Constraints(
      2.0, // Max velocity in meters per second
      1.0  // Max acceleration in meters per second squared
    );

    // Profiled PID Controller with constraints
    profiledPIDController = new ProfiledPIDController(
      1.0, // P
      0.0, // I 
      0.0, // D
      constraints // Motion constraints
    );

    // tolerance
    profiledPIDController.setTolerance(0.02);

    // kS - static friction
    // kG - gravity compensation
    // kV - velocity feedforward
    // kA - acceleration feedforward
    feedforward = new ElevatorFeedforward(0.0, 0.0, 0.0, 0.0); // Values obtained from SysID

    elevatorRoutine = new ElevatorRoutine(this);

    levelOne = new DigitalInput(Constants.ElevatorConstants.levelOneDIO);
    levelTwo = new DigitalInput(Constants.ElevatorConstants.levelTwoDIO);
    levelThree = new DigitalInput(Constants.ElevatorConstants.levelThreeDIO);
    levelFour = new DigitalInput(Constants.ElevatorConstants.levelFourDIO);
  }

  public void setSpeed(double speed) {
    elevatorMotor.set(speed);
  }

  public void setGoal(double position) {
    profiledPIDController.setGoal(position);
  }

  public boolean atGoal() {
    return profiledPIDController.atGoal();
  }

  public void setPosition(double position) {
    setpoint = position;
  }

  public double getPosition() {
    return elevatorMotor.getEncoder().getPosition();
  }

  public double getVelocity() {
    return elevatorMotor.getEncoder().getVelocity();
  }

  public void stop() {
    elevatorMotor.set(0);
  }

  public void setVoltage(double volts) {
    elevatorMotor.setVoltage(volts);
  }

  public void setCalculatedVoltage() {
    // Get the next profile state from the motion profile
    double pidOutput = profiledPIDController.calculate(getPosition());

    // Get the feedforward output based on the profile's velocity and acceleration
    TrapezoidProfile.State setpoint = profiledPIDController.getSetpoint();
    double feedforwardOutput = feedforward.calculate(setpoint.position, setpoint.velocity);

    // Outputs Combined
    double output = pidOutput + feedforwardOutput;

    // Output applied to the motor
    elevatorMotor.setVoltage(output);
  }

  @Override
  public void periodic() {
  }
}
