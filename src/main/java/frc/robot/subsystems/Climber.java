// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DIOConstants;

public class Climber extends SubsystemBase {

    public CANSparkMax climberMotor;
    public PIDController climberPIDController;
    public DigitalInput bottomHallEffect;

    /** Creates a new Climber. */
    public Climber() {

        climberMotor = new CANSparkMax(ClimberConstants.climberWinchMotorID, MotorType.kBrushless);
        climberMotor.setIdleMode(IdleMode.kBrake);
        bottomHallEffect = new DigitalInput(DIOConstants.climberHallEffectDIO);

    }

    public CANSparkMax getClimberMotor() {
        return climberMotor;
    }

    public void setClimberMotorSpeed(double output) {
        climberMotor.set(output);
    }

    public PIDController getClimberController() {
        return climberPIDController;
    }

    public boolean isBottomLimitReached() {
        return !bottomHallEffect.get();
    }

    @Override
    public void periodic() {

        SmartDashboard.putBoolean("is bottom limit climber reached", isBottomLimitReached());

        // if (isTopLimitReached()) {
        // climberMotor.set(0);
        // }

        // This method will be called once per scheduler run
    }
}
