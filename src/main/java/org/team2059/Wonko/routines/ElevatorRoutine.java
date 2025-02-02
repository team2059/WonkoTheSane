// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2059.Wonko.routines;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.team2059.Wonko.subsystems.Elevator;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ElevatorRoutine {
    private final SysIdRoutine sysIdRoutine;

    private final MutVoltage elevatorRoutineAppliedVoltage = Volts.mutable(0);
    private final MutDistance elevatorRoutineDistance = Meters.mutable(0);
    private final MutLinearVelocity elevatorRoutineVelocity = MetersPerSecond.mutable(0);

    public ElevatorRoutine(Elevator elevator) {
        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1).per(Units.Second),  // Ramp rate of 1 volt per second
                Volts.of(2),                    // Maximum voltage of 2 volts       
                Time.ofBaseUnits(2, Units.Second), // Test duration of 2 seconds
                null 
            ),
            new SysIdRoutine.Mechanism(
                voltage -> {
                    elevator.setVoltage(voltage.in(Volts));
                },
                log -> {
                    // Log position and velocity for SysID
                    log.motor("elevator-motor")
                        .voltage(elevatorRoutineAppliedVoltage.mut_replace(
                            elevator.elevatorMotor.getAppliedOutput() * elevator.elevatorMotor.getBusVoltage(), 
                            Volts))
                        .linearPosition(elevatorRoutineDistance.mut_replace(
                            elevator.getPosition(), 
                            Meters))
                        .linearVelocity(elevatorRoutineVelocity.mut_replace(
                            elevator.getVelocity(), 
                            MetersPerSecond));
                },
                // Tell SysId to make generated commands require this subsystem, suffix test state in
                // WPILog with this subsystem's name
                elevator
            ));
    }

    // Returns a command that will execute a quasistatic test in the given direction
    public Command quasistaticForward() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    }
    
    public Command quasistaticReverse() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
    }

    // Returns a command that will execute a dynamic test in the given direction
    public Command dynamicForward() {
        return sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
    }
    
    public Command dynamicReverse() {
        return sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);
    }
}