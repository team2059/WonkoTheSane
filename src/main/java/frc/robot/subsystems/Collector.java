// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CollectorConstants;
import frc.robot.Constants.DIOConstants;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Collector extends SubsystemBase {

    public CANSparkMax rollerMotor;
    public CANSparkMax tiltMotor;

    public DutyCycleEncoder tiltThruBore;

    public DigitalInput collectorNoteSensor = new DigitalInput(DIOConstants.collectorOpticalDIO);

    /** Creates a new Collector. */
    public Collector() {

        rollerMotor = new CANSparkMax(CollectorConstants.collectorRollerID, MotorType.kBrushless);
        tiltMotor = new CANSparkMax(CollectorConstants.collectorTiltID,
                MotorType.kBrushless);
        tiltMotor.setInverted(false);
        tiltMotor.setIdleMode(IdleMode.kBrake);
        rollerMotor.setInverted(false);

        tiltThruBore = new DutyCycleEncoder(DIOConstants.collectorTiltThruBoreDIO);

    }

    @Override
    public void periodic() {

        Logger.recordOutput("collecter note sensor", isNotePresent());

        SmartDashboard.putBoolean("HAS NOTE?", isNotePresent());

        // if (isNotePresent()) {
        // rollerMotor.set(0);
        // }

        // double setpoint = 0.5;

        // "goal" = setpoint
        // double pidOutputValue =
        // tiltPidController.calculate(tiltThruBore.getAbsolutePosition(), 0.5);
        // double pidOutputValue = new PIDController(1.5, 0,
        // 0).calculate(tiltThruBore.getAbsolutePosition(), setpoint);
        // double ffOutputValue = new SimpleMotorFeedforward(0, 0,
        // 0).calculate(tiltPidController.getSetpoint().position,
        // tiltPidController.getSetpoint().velocity);

        // Logger.recordOutput("setpoint", setpoint);
        // Logger.recordOutput("pidOutputValue", pidOutputValue);
        // Logger.recordOutput("ffOutputValue", ffOutputValue);
        Logger.recordOutput("collector thrubore pos",
                tiltThruBore.getAbsolutePosition());

        // tiltMotor.set(pidOutputValue);

        // tiltMotor.setVoltage(pidOutputValue);

        // This method will be called once per scheduler run
    }

    public void setRollerMotor(double speed) {
        rollerMotor.set(speed);
    }

    public boolean isNotePresent() {
        return !collectorNoteSensor.get();

    }
}
