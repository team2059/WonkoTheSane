// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CollectorCmds;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CollectorConstants;
import frc.robot.subsystems.Collector;

public class TiltCollectorToSetpointONLYCLIMBCmd extends Command {
    Collector collector;
    double setpoint;
    PIDController tiltController = new PIDController(CollectorConstants.tiltkP, 0, CollectorConstants.tiltkD);

    /** Creates a new MoveCollectorCmd. */
    public TiltCollectorToSetpointONLYCLIMBCmd(Collector collector, double setpoint) {
        this.collector = collector;
        this.setpoint = setpoint;

        addRequirements(collector);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        tiltController.setTolerance(0.02);

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        double pidOutout = tiltController.calculate(collector.tiltThruBore.getAbsolutePosition(),
                setpoint);

        Logger.recordOutput("pid output", pidOutout);
        Logger.recordOutput("setpoint", setpoint);

        collector.tiltMotor.set(pidOutout);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
