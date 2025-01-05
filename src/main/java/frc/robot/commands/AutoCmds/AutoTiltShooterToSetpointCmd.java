// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCmds;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class AutoTiltShooterToSetpointCmd extends Command {
    Shooter shooter;
    double setpoint;
    // PIDController tiltController = new PIDController(2, 0, 0.05);

    ProfiledPIDController tiltController = new ProfiledPIDController(0.015,
            0, 0.0,
            new TrapezoidProfile.Constraints(180, 135));

    /** Creates a new MoveCollectorCmd. */

    public AutoTiltShooterToSetpointCmd(Shooter shooter, double setpoint) {
        this.shooter = shooter;
        this.setpoint = setpoint;

        // addRequirements(shooter);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        tiltController.setTolerance(1.5);
        tiltController.reset(shooter.getAbsoluteShooterTiltPosDegrees());
        // tiltController.enableContinuousInput(-90, 90);

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // double pidOutout = tiltController.calculate(shooter.getShooterTiltPos(),
        // setpoint);

        double pidOutout = tiltController.calculate(shooter.getAbsoluteShooterTiltPosDegrees(),
                new State(setpoint, 0));

        shooter.shooterTiltMotor.set(MathUtil.clamp(pidOutout, -1.0, 1.0));

        Logger.recordOutput("tilt pidOutout", pidOutout);
        Logger.recordOutput("tilt setpoint", setpoint);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

        // shooter.shooterTiltMotor.set(0);

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        return false;
    }
}
