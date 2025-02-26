package org.team2059.Wonko.routines;

import static edu.wpi.first.units.Units.*;

import org.team2059.Wonko.Constants.AlgaeCollectorConstants;
import org.team2059.Wonko.subsystems.algae.AlgaeCollector;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class AlgaeCollectorRoutine {
    private final AlgaeCollector algaeCollector;

    private final SysIdRoutine sysIdRoutine;

    private final MutVoltage appliedVoltage = Volts.mutable(0);
    private final MutAngle angle = Radians.mutable(0);
    private final MutAngularVelocity angularVelocity = RadiansPerSecond.mutable(0);

    public AlgaeCollectorRoutine(AlgaeCollector algaeCollector) {
        this.algaeCollector = algaeCollector;

        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.3).per(Second), // Ramp rate in volts per second
                Volts.of(1), // Dynamic step voltage
                Time.ofBaseUnits(4, Second), // Test duration in seconds
                null
            ), 
            new SysIdRoutine.Mechanism(
                voltage -> {
                    algaeCollector.io.setTiltVolts(voltage.in(Volts));
                }, 
                log -> {
                    log.motor("algaecollector-tiltmotor")
                        .voltage(appliedVoltage.mut_replace(algaeCollector.inputs.tiltMotorAppliedVolts, Volts))
                        .angularPosition(angle.mut_replace(algaeCollector.inputs.integratedTiltPosRadians, Radians))
                        .angularVelocity(angularVelocity.mut_replace(algaeCollector.inputs.integratedTiltVelRadPerSec, RadiansPerSecond));
                }, 
                algaeCollector
            )
        );
    }

    // Quasistatic tests in given direction
    public Command quasistaticForward() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)
            .until(() -> algaeCollector.inputs.thruBorePositionRadians >= AlgaeCollectorConstants.thruBoreMaxmimum.in(Radians) - 0.1);
    }
    public Command quasistaticReverse() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse)
            .until(() -> algaeCollector.inputs.thruBorePositionRadians <= AlgaeCollectorConstants.thruBoreMinimum.in(Radians) + 0.1);
    }

    // Dynamic tests in given direction
    public Command dynamicForward() {
        return sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward)
            .until(() -> algaeCollector.inputs.thruBorePositionRadians >= AlgaeCollectorConstants.thruBoreMaxmimum.in(Radians) - 0.1);
    }
    public Command dynamicReverse() {
        return sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse)
            .until(() -> algaeCollector.inputs.thruBorePositionRadians <= AlgaeCollectorConstants.thruBoreMinimum.in(Radians) + 0.1);
    }
}