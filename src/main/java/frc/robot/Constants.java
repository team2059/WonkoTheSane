// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosly.
 */
public final class Constants {
  public static class OperatorConstants {

    /* ===== */
    /* PORTS */
    /* ===== */

    public static final int LogitechControllerPort = 0;

    /* ==== */
    /* AXES */
    /* ==== */

    public static final int JoystickTranslationAxis = 1;
    public static final int JoystickStrafeAxis = 0;
    public static final int JoystickRotationAxis = 2;
    public static final int JoystickSliderAxis = 3;

    /* ======= */
    /* BUTTONS */
    /* ======= */

    public static final int JoystickResetHeading = 5;
    public static final int JoystickRobotRelative = 3;
    public static final int JoystickIntakeCoral = 6;
    public static final int JoystickReleaseCoral = 4;
    public static final int JoystickIntakeAlgae = 0;
    public static final int JoystickReleaseAlgae = 0;
  }

  public static class SwerveConstants {

    public static final double wheelBase = Units.inchesToMeters(24.5); // distance between front wheels (like train track)
    public static final double trackWidth = Units.inchesToMeters(18.5); // distance from center of wheels on side

    public static final double wheelDiameter = Units.inchesToMeters(4.0 / 1);

    public static final Translation2d flModuleOffset = new Translation2d(trackWidth / 2.0, wheelBase / 2.0);
    public static final Translation2d frModuleOffset = new Translation2d(trackWidth / 2.0, -wheelBase / 2.0);
    public static final Translation2d blModuleOffset = new Translation2d(-trackWidth / 2.0, wheelBase / 2.0);
    public static final Translation2d brModuleOffset = new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0);

    /* =========== */
    /* GEAR RATIOS */
    /* =========== */

    /*
     * MK4i L2:
     * - Drive 6.75:1
     * - Rotation 150:7
     * 
     * MK4n L2:
     * - Drive 5.9:1
     * - Rotation 75:4
     */
    public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
    public static final double rotationGearRatio = (150.0 / 7.0); // 150:7

    /* ================== */
    /* CONVERSION FACTORS */
    /* ================== */

    // Given Motor Rotations, convert to Meters traveled
    public static final double driveEncoderPositionConversionFactor = (Math.PI * Units.inchesToMeters(wheelDiameter)) / (driveGearRatio);
    // Given Motor RPM, convert to Meters/second
    public static final double driveEncoderVelocityConversionFactor = driveEncoderPositionConversionFactor / 60.0;
    // Given Motor Rotations, convert to Radians traveled
    public static final double rotationEncoderPositionConversionFactor = (2.0 * Math.PI) / rotationGearRatio;
    // Given Motor RPM, convert to Radians/second
    public static final double rotationEncoderVelocityConversionFactor = rotationEncoderPositionConversionFactor / 60.0;

    /* ============== */
    /* SWERVE MODULES */
    /* ============== */

    /*
     * CAN IDs: found and set via REV hardware client
     * CANcoder Offsets: found in Phoenix Tuner X as "Absolute position"
     *  after manually straightening wheel (converted to radians here)
     */

    // front left
    public static final int frontLeftDriveMotorId = 1;
    public static final int frontLeftRotationMotorId = 2;
    public static final int frontLeftCanCoderId = 11;
    public static final double frontLeftOffsetRad = 0.862061 * 2 * Math.PI;
    // front right
    public static final int frontRightDriveMotorId = 8;
    public static final int frontRightRotationMotorId = 7;
    public static final int frontRightCanCoderId = 12;
    public static final double frontRightOffsetRad = 0.017578 * 2 * Math.PI;
    // back left
    public static final int backLeftDriveMotorId = 5;
    public static final int backLeftRotationMotorId = 6;
    public static final int backLeftCanCoderId = 14;
    public static final double backLeftOffsetRad = 0.244141 * 2 * Math.PI;
    // back right
    public static final int backRightDriveMotorId = 4;
    public static final int backRightRotationMotorId = 3;
    public static final int backRightCanCoderId = 13;
    public static final double backRightOffsetRad = 0.490234 * 2 * Math.PI;

    /* ======== */
    /* MAXIMUMS */
    /* ======== */

    // Global maximums
    public static final double maxVelocity = 4.5; // meters/sec
    public static final double maxAcceleration = 10; // meters/sec^2
    public static final double maxAngularVelocity = 2 * Math.PI; // rad/sec
    public static final double maxAngularAcceleration = 4 * Math.PI; // rad/sec^2
    // Teleop max speeds
    public static final double kTeleDriveMaxSpeed = 7.5 / 4.0;
    public static final double kTeleDriveMaxAngularSpeed = 3;

    /* =============================== */
    /* SWERVE MODULE CONTROL CONSTANTS */
    /* =============================== */
    
    public static final double kPRotation = 0.25;
    // kS: voltage needed to overcome static friction
    // kV: voltage needed to run at constant velocity
    // kA: voltage needed to accelerate
    public static final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0.2, 2.5, 0.0);
  }

  public static class AutoConstants {
    public static final PIDConstants autoTranslationConstants = new PIDConstants(0, 0, 0);
    public static final PIDConstants autoRotationConstants = new PIDConstants(0, 0, 0);
  }

  public static class VisionConstants {
    public static final String cameraName = "hhCam";
    public static final double cameraHeightMeters = Units.inchesToMeters(21.875);
    public static final double cameraPitchRadians = Math.toRadians(0);
    public static final double targetHeightMeters = Units.inchesToMeters(28.25);
  }

  public static class CoralIntakeConstants {
    public static final int intakeMotorID = 9;
  }

  public static class AlgaeIntakeConstants {
    public static final int motor1ID = 10;
    public static final int motor2ID = 11;
  }
}
