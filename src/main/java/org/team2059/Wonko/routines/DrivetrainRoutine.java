package org.team2059.Wonko.routines;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.team2059.Wonko.subsystems.Drivetrain;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class DrivetrainRoutine {

    private final SysIdRoutine sysIdRoutine;

    // Mutable holders for unit-safe voltage, linear distance, and linear velocity values, persisted to avoid reallocation.
    private final MutVoltage driveRoutineAppliedVoltage = Volts.mutable(0);
    private final MutDistance driveRoutineDistance = Meters.mutable(0);
    private final MutLinearVelocity driveRoutineVelocity = MetersPerSecond.mutable(0);

    public DrivetrainRoutine(Drivetrain drivetrain) {
        // SysID characterization configuration
        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1).per(Units.Second), // Ramp rate in V/s
                Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
                Time.ofBaseUnits(5.0, Units.Second), // Use default timeout of 10 sec
                null
            ),
            new SysIdRoutine.Mechanism(
                // Tell SysID how to plumb the driving voltage to the motors
                voltage -> {
                drivetrain.frontLeft.characterizeDriveVoltage(voltage.in(Volts));
                drivetrain.frontRight.characterizeDriveVoltage(voltage.in(Volts));
                drivetrain.backLeft.characterizeDriveVoltage(voltage.in(Volts));
                drivetrain.backRight.characterizeDriveVoltage(voltage.in(Volts));
                }, 
                // Tell SysID how to record a frame of data for each motor on the mechanism
                log -> {
                log.motor("drive-frontleft")
                    .voltage(driveRoutineAppliedVoltage.mut_replace(drivetrain.frontLeft.getDriveMotor().getAppliedOutput() * drivetrain.frontLeft.getDriveMotor().getBusVoltage(), Volts))
                    .linearPosition(driveRoutineDistance.mut_replace(drivetrain.frontLeft.getDriveEncoderPosition(), Meters))
                    .linearVelocity(driveRoutineVelocity.mut_replace(drivetrain.frontLeft.getDriveVelocity(), MetersPerSecond));

                log.motor("drive-frontright")
                    .voltage(driveRoutineAppliedVoltage.mut_replace(drivetrain.frontRight.getDriveMotor().getAppliedOutput() * drivetrain.frontRight.getDriveMotor().getBusVoltage(), Volts))
                    .linearPosition(driveRoutineDistance.mut_replace(drivetrain.frontRight.getDriveEncoderPosition(), Meters))
                    .linearVelocity(driveRoutineVelocity.mut_replace(drivetrain.frontRight.getDriveVelocity(), MetersPerSecond));

                log.motor("drive-backleft")
                    .voltage(driveRoutineAppliedVoltage.mut_replace(drivetrain.backLeft.getDriveMotor().getAppliedOutput() * drivetrain.backLeft.getDriveMotor().getBusVoltage(), Volts))
                    .linearPosition(driveRoutineDistance.mut_replace(drivetrain.backLeft.getDriveEncoderPosition(), Meters))
                    .linearVelocity(driveRoutineVelocity.mut_replace(drivetrain.backLeft.getDriveVelocity(), MetersPerSecond));

                log.motor("drive-backright")
                    .voltage(driveRoutineAppliedVoltage.mut_replace(drivetrain.backRight.getDriveMotor().getAppliedOutput() * drivetrain.backRight.getDriveMotor().getBusVoltage(), Volts))
                    .linearPosition(driveRoutineDistance.mut_replace(drivetrain.backRight.getDriveEncoderPosition(), Meters))
                    .linearVelocity(driveRoutineVelocity.mut_replace(drivetrain.backRight.getDriveVelocity(), MetersPerSecond));
                }, 
                // Tell SysId to make generated commands require this subsystem, suffix test state in
                // WPILog with this subsystem's name
                drivetrain
            )
        );
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
