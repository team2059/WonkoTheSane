// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2059.Wonko.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import org.team2059.Wonko.util.LoggedTunableNumber;
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
  enum elevatorPositionEnum {
    L0,
    L1,
    L2,
    L3,
    L4,
  }
  
  private elevatorPositionEnum elevatorPosition;
  public final SparkMax elevatorMotor;

  // Motion Profiling, PID, Feedforward
  private final ProfiledPIDController profiledPIDController;
  private final ElevatorFeedforward feedforward;
  private final TrapezoidProfile.Constraints constraints;
  private final ElevatorRoutine elevatorRoutine;

  // Limit Switches
  private final DigitalInput levelOne, levelTwo, levelThree, levelFour;

  // Tunable Numbers for PID
  private final LoggedTunableNumber elevatorP = new LoggedTunableNumber("Elevator/P", 0.01);
  private final LoggedTunableNumber elevatorD = new LoggedTunableNumber("Elevator/D", 0);
  private final LoggedTunableNumber tolerance = new LoggedTunableNumber("Elevator/Tolerance", 0.02);
  public PIDController turnController = new PIDController(elevatorP.get(), 0, elevatorD.get());

  /** Creates a new Elevator. */
  public Elevator() {
    elevatorMotor = new SparkMax(Constants.ElevatorConstants.elevatorMotorID, MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(true);
    config.idleMode(IdleMode.kBrake);
    config.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
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
    turnController.setTolerance(0.02);

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

  public void goToSetpoint(double setpoint) {
    double output = turnController.calculate(elevatorMotor.getEncoder().getPosition(), setpoint);

    elevatorMotor.set(output);
  }

  // 
  private void goToZero() {
    // 
    // TODO:
    // move down until hit limit switch 0, unless already at 0
    elevatorPosition = elevatorPositionEnum.L0;
  }

  @Override
  public void periodic() {
    if (elevatorP.hasChanged(hashCode()) || elevatorD.hasChanged(hashCode()) || tolerance.hasChanged(hashCode())) {
      turnController.setP(elevatorP.get());
      turnController.setD(elevatorD.get());
      turnController.setTolerance(tolerance.get());
    }
    Logger.recordOutput("Elevator/RelativeEncoder", getPosition());

    
  }
}
