package org.team2059.Wonko.routines;

import static edu.wpi.first.units.Units.*;

import org.team2059.Wonko.subsystems.elevator.Elevator;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ElevatorRoutine {
    private final Elevator elevator;

    private final SysIdRoutine sysIdRoutine;

    private final MutVoltage appliedVoltage = Volts.mutable(0);
    private final MutDistance distance = Meters.mutable(0);
    private final MutLinearVelocity linearVelocity = MetersPerSecond.mutable(0);

    public ElevatorRoutine(Elevator elevator) {
        this.elevator = elevator;

        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(3).per(Units.Second), // Ramp rate in volts per second
                Volts.of(4), // Dynamic step voltage
                Time.ofBaseUnits(10, Units.Second), // Test duration of 2 seconds
                null
            ), 
            new SysIdRoutine.Mechanism(
                voltage -> {
                    elevator.io.setVoltage(voltage.in(Volts));
                }, 
                log -> {
                    log.motor("elevator-motor")
                        .voltage(appliedVoltage.mut_replace(elevator.inputs.rightMotorAppliedVolts, Volts))
                        .linearPosition(distance.mut_replace(elevator.inputs.positionMeters, Meters))
                        .linearVelocity(linearVelocity.mut_replace(elevator.inputs.velocityMetersPerSecond, MetersPerSecond));
                }, 
                elevator
            )
        );
    }

    // Quasistatic tests in given direction
    public Command quasistaticForward() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).until(() -> elevator.inputs.positionMeters >= 2);
    }
    public Command quasistaticReverse() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).until(() -> elevator.inputs.positionMeters <= 0.1);
    }

    // Dynamic tests in given direction
    public Command dynamicForward() {
        return sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).until(() -> elevator.inputs.positionMeters >= 2);
    }
    public Command dynamicReverse() {
        return sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).until(() -> elevator.inputs.positionMeters <= 0.1);
    }
}
