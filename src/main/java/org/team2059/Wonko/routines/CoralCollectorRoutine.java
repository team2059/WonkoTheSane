package org.team2059.Wonko.routines;

import static edu.wpi.first.units.Units.*;

import org.team2059.Wonko.Constants.CoralCollectorConstants;
import org.team2059.Wonko.subsystems.coral.CoralCollector;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/*
 * SysID routine for characterizing the coral tilt mechanism
 * Tests are run until the max/min is hit (specified in constants), then immediately stopped.
 */
public class CoralCollectorRoutine {
    private final CoralCollector coralCollector;

    private final SysIdRoutine sysIdRoutine;

    private final MutVoltage appliedVoltage = Volts.mutable(0);
    private final MutAngle angle = Radians.mutable(0);
    private final MutAngularVelocity angularVelocity = RadiansPerSecond.mutable(0);

    public CoralCollectorRoutine(CoralCollector coralCollector) {
        this.coralCollector = coralCollector;

        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.5).per(Units.Second), // Ramp rate in volts per second
                Volts.of(2), // Dynamic step voltage
                Time.ofBaseUnits(4, Units.Second), // Test duration in seconds
                null
            ),
            new SysIdRoutine.Mechanism(
                voltage -> {
                    coralCollector.io.setTiltVolts(voltage.in(Volts));
                }, 
                log -> {
                    log.motor("coralcollector-tiltmotor")
                        .voltage(appliedVoltage.mut_replace(coralCollector.inputs.tiltMotorAppliedVolts, Volts))
                        .angularPosition(angle.mut_replace(coralCollector.inputs.tiltMotorPositionRad, Radians))
                        .angularVelocity(angularVelocity.mut_replace(coralCollector.inputs.tiltMotorVelocityRadPerSec, RadiansPerSecond));
                }, 
                coralCollector
            )
        );
    }

    // Quasistatic tests in given direction
    public Command quasistaticForward() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)
            .until(() -> coralCollector.inputs.thruBorePositionRadians >= CoralCollectorConstants.thruBoreMaxmimum.in(Radians));
    }
    public Command quasistaticReverse() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse)
            .until(() -> coralCollector.inputs.thruBorePositionRadians <= CoralCollectorConstants.thruBoreMinimum.in(Radians));
    }

    // Dynamic tests in given direction
    public Command dynamicForward() {
        return sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward)
            .until(() -> coralCollector.inputs.thruBorePositionRadians >= CoralCollectorConstants.thruBoreMaxmimum.in(Radians));
    }
    public Command dynamicReverse() {
        return sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse)
            .until(() -> coralCollector.inputs.thruBorePositionRadians <= CoralCollectorConstants.thruBoreMinimum.in(Radians));
    }
}
