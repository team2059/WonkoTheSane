// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2059.Wonko;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;

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

    // Sets whether or not tunable numbers can be changed. If false, only defaults will be used.
    public static final boolean tuningMode = true;

    /* ===== */
    /* PORTS */
    /* ===== */

    public static final int logitechPort = 0;
    public static final int buttonBoxPort = 1;
    public static final int xboxControllerPort = 2;

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

    // We'll add these later once the button box is finalized
    public static final int JoystickResetHeading = 5;
    public static final int JoystickRobotRelative = 6;
    public static final int JoystickInvertedDrive = 4;
    public static final int JoystickStrafeOnly = 3;
  }

  public static class ClimberConstants {
    // CAN IDs 
    // (one left, one right motor, they are supposed to behave exactly the same but INVERTED!)
    public static final int motor1ID = 17;
    public static final int motor2ID = 18; 

    // REV thrubore encoder port
    public static final int climbThroughBoreDIO = 8; 

    // Hard limits, as reported by thrubore
    public static final Angle upperLimit = Radians.of(5.76);
    public static final Angle lowerLimit = Radians.of(3);
  }

  public static class DrivetrainConstants {

    public static final Distance wheelBase = Inches.of(24.75); // Distance from center of wheels on side
    public static final Distance trackWidth = Inches.of(20.75); // Distance between front wheels (like train track)

    // Diameter of swerve module wheel
    public static final Distance wheelDiameter = Inches.of(4.0);

    // Kinematics gets each module relative to center. X is forward/backward and Y is left/right.
    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      new Translation2d(wheelBase.in(Meters) / 2.0, trackWidth.in(Meters) / 2.0), // front right (+,+)
      new Translation2d(wheelBase.in(Meters) / 2.0, -trackWidth.in(Meters) / 2.0), // back right (+,-)
      new Translation2d(-wheelBase.in(Meters) / 2.0, trackWidth.in(Meters) / 2.0), // front left (-,+)
      new Translation2d(-wheelBase.in(Meters) / 2.0, -trackWidth.in(Meters) / 2.0) // back left (-,-)
    );

    /* =========== */
    /* GEAR RATIOS */
    /* =========== */

    public static final double driveGearRatio = (5.9 / 1.0); // 5.9:1
    public static final double rotationGearRatio = (18.75 / 1.0); // 18.75

    /* ================== */
    /* CONVERSION FACTORS */
    /* ================== */

    // Given Motor Rotations, convert to Meters traveled
    // (pi * d) / (Gear Ratio)
    // where d is wheel diameter, in meters
    public static final double driveEncoderPositionConversionFactor = 0.05409929044;
    // Given Motor RPM, convert to Meters/second
    public static final double driveEncoderVelocityConversionFactor = driveEncoderPositionConversionFactor / 60.0;
    // Given Motor Rotations, convert to Radians
    // (2 * pi) / (Gear Ratio)
    public static final double rotationEncoderPositionConversionFactor = 0.3351032164;
    // Given Motor RPM, convert to Radians/second
    public static final double rotationEncoderVelocityConversionFactor = rotationEncoderPositionConversionFactor / 60.0;

    /* ============== */
    /* SWERVE MODULES */
    /* ============== */

    /*
     * CAN IDs: found and set via REV hardware client
     * CANcoder Offsets: found in Phoenix Tuner X as "Absolute position"
     *  after manually straightening wheel (converted to radians here by multiplying by 2pi)
     */

    // front left
    public static final int frontLeftDriveMotorId = 1;
    public static final int frontLeftRotationMotorId = 2;
    public static final int frontLeftCanCoderId = 10;
    public static final double frontLeftOffsetRad = 0.416260 * 2 * Math.PI;
    // front right
    public static final int frontRightDriveMotorId = 3;
    public static final int frontRightRotationMotorId = 4;
    public static final int frontRightCanCoderId = 20;
    public static final double frontRightOffsetRad = 0.978760 * 2 * Math.PI;
    // back left
    public static final int backLeftDriveMotorId = 5;
    public static final int backLeftRotationMotorId = 6;
    public static final int backLeftCanCoderId = 30;
    public static final double backLeftOffsetRad = 0.831787 * 2 * Math.PI;
    // back right
    public static final int backRightDriveMotorId = 7;
    public static final int backRightRotationMotorId = 8;
    public static final int backRightCanCoderId = 40;
    public static final double backRightOffsetRad = 0.136719 * 2 * Math.PI;

    /* ======== */
    /* MAXIMUMS */
    /* ======== */

    // Global maximums
    public static final double maxVelocity = 5; // meters/sec
    public static final double maxAcceleration = 10; // meters/sec^2
    public static final double maxAngularVelocity = 2 * Math.PI; // rad/sec
    public static final double maxAngularAcceleration = 4 * Math.PI; // rad/sec^2
    // Teleop max speeds
    public static final double kTeleDriveMaxSpeed = 4;
    public static final double kTeleDriveMaxAngularSpeed = Math.PI;

    /* =============================== */
    /* SWERVE MODULE CONTROL CONSTANTS */
    /* =============================== */
    
    public static final double kPRotation = 0.25;
    // kS: voltage needed to overcome static friction
    // kV: voltage needed to run at constant velocity
    // kA: voltage needed to accelerate
    public static final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0.17821, 1.9047, 0.14686);
    public static final SimpleMotorFeedforward turnFF = new SimpleMotorFeedforward(0, 0, 0);
  }

  public static class AutoConstants {

    // Feedback constants for x & y translation in auto.
    public static final double kAutoTranslationP = 5.0;
    public static final double kAutoTranslationD = 0;

    // Feedback constants for theta (rotation) in auto.
    public static final double kAutoRotationP = 5.0;
    public static final double kAutoRotationD = 0.0;

    /* FOR ROBOTCONFIG AUTO STUFF... */
    /* Not used right now. */
    // public static final double kMass = 30;
    // public static final double kMomentOfIntertia = 3;
    
    // // CoF taken from https://www.chiefdelphi.com/t/coefficient-of-friction/467778
    // public static final double kWheelCoF = 1.542; // Coefficient of friction of wheels
    // public static final int driveCurrentLimit = 40;
    // public static final int turnCurrentLimit = 20;
  }

  public static class VisionConstants {

    // Cam names set using Arducam serial number utility. On DS PC.
    public static final String upperCameraName = "Bcam9782";
    public static final String lowerCameraName = "Acam9782";
    
    public static final Transform3d upperCameraToRobot = 
      new Transform3d(
        new Translation3d(Units.inchesToMeters(6.75), Units.inchesToMeters(8.25), Units.inchesToMeters(38.875)), 
        new Rotation3d(0, Units.degreesToRadians(-24), Units.degreesToRadians(0))
      );
    
    public static final Transform3d lowerCameraToRobot = 
      new Transform3d(
        new Translation3d(Units.inchesToMeters(14), Units.inchesToMeters(0), Units.inchesToMeters(10.5)), 
        new Rotation3d(0, 0, 0)
      );


    /* 
     * Tags of each reef side. 
     * Starts at the side closest to the driver station
     * Goes clockwise (relative to driver station)
     */
    public static final ArrayList<Integer> redReefTags = new ArrayList<>(Arrays.asList(7, 6, 11, 10, 9, 8));
    public static final ArrayList<Integer> blueReefTags = new ArrayList<>(Arrays.asList(18, 19, 20, 21, 22, 17));
    public static ArrayList<Integer> reefTags = new ArrayList<>();

    // Tags of human player stations 
    // Starts at left human player station from driver POV
    public static final ArrayList<Integer> redHPTags = new ArrayList<>(Arrays.asList(1, 2));
    public static final ArrayList<Integer> blueHPTags = new ArrayList<>(Arrays.asList(13, 12));
    public static ArrayList<Integer> HPTags = new ArrayList<>();

    // Standard deviations below are from Team Spectrum 3847’s X-Ray robot

    /**
     * Standard deviations of model states. Increase these numbers to trust your
     * model's state estimates less. This matrix is in the form [x, y, theta]ᵀ,
     * with units in meters and radians, then meters.
     */
    public static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 10);

    /**
     * Standard deviations of the vision measurements. Increase these numbers to
     * trust global measurements from vision less. This matrix is in the form
     * [x, y, theta]ᵀ, with units in meters and radians.
     */
    public static final Matrix<N3, N1> measurementStdDevs = VecBuilder.fill(5, 5, 500);
    
    // The standard deviations of our vision estimated poses, which affect correction rate
    public static final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(2, 2, 8);
    public static final Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  
    public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    public static final Distance reefAlignFrontOffset = Inches.of(18);
    public static final Distance reefAlignLeftStrafeOffset = Inches.of(-4);
    public static final Distance reefAlignRightStrafeOffset = Inches.of(4);

    public static final double reefXOffsetInches = 15;
    public static final double reefYRightOffsetInches = 5;
    public static final double reefYLeftOffsetInches = -7;

    public static final double hpXOffsetInches = 15;
    public static final double hpYOffsetInches = 0;
  }

  public static class AlgaeCollectorConstants {

    // CAN IDs
    public static final int motor1Id = 11;
    public static final int motor2Id = 12;
    public static final int tiltMotorId = 13;

    // DIO IDs
    public static final int irSensorDio = 7;

    public static final double horizontalOffset = -2.16; // Add this value to the raw thrubore reading to have 0 reported at the horizontal.

    // Conv. factors (2pi/GR to get Radians)
    public static final double tiltMotorPositionConvFactor = 0.1396263402;
    public static final double tiltMotorVelocityConvFactor = 0.00232710567;

    // Hard limits
    public static final Angle thruBoreMinimum = Radians.of(0);
    public static final Angle thruBoreMaximum = Radians.of(1.2);

    public static final Current stallDetection = Amps.of(10);
  }

  public static class CoralCollectorConstants {

    // CAN IDs
    public static final int intakeMotorId = 14;
    public static final int tiltMotorId = 15;

    // DIO port IDs
    public static final int irSensorDio = 6;

    // Conv. factors
    // 2pi/GR
    public static final double tiltPositionConversionFactor = 0.2513274123; // Rotations -> radians w/ 9:1 g.r.
    public static final double tiltVelocityConversionFactor = 0.004188790205; // RPM -> rad/sec "

    // PID & FF gains
    public static final double kPIntake = 0.000019;
    public static final double kFIntake = 0.000149;
    public static final double kPTilt = 0.3;
    public static final double kITilt = 0.0;
    public static final double kDTilt = 0.0;
    public static final double kSTilt = 0.0; // Note concerning FF values: following mechanical changes, constants no longer needed.
    public static final double kVTilt = 0.0; //  , center of gravity has shifted requiring not as much control complication
    public static final double kATilt = 0.0;
    public static final double kGTilt = 0.0;

    // Trapezoidal constraints
    public static final double tiltMaxVelocity = 7;
    public static final double tiltMaxAccel = 5;

    public static final double horizontalOffset = 0.358; // Add this value to the raw thrubore reading to have 0 reported at the horizontal.

    // Smart current limit for Spark controller
    public static final Current tiltCurrentLimit = Amps.of(30);

    // Hard limits
    public static final Angle thruBoreMinimum = Radians.of(-1.6);
    public static final Angle thruBoreMaxmimum = Radians.of(Math.PI / 2.0);

    // Angular setpoints
    public static final Angle restingCoralCollectorPos = Radians.of(1.36);
    public static final Angle[] levelCoralTiltAngle = {
      restingCoralCollectorPos,  // L0
      restingCoralCollectorPos,  // L1
      Radians.of(-0.657),        // L2
      Radians.of(-0.74),        // L3
      Radians.of(-1.06)         // L4
    };
    public static final Angle humanPlayerAngle = Radians.of(0.5);

  }

  public static class ElevatorConstants {

    // CAN ID
    public static final int motorId = 16;

    // Smart current limit for Spark motor
    public static final Current currentLimit = Amps.of(40);

    // MAX HEIGHT IS A HARD LIMIT! DO NOT GO TOO CLOSE
    public static final Distance maxHeight = Distance.ofBaseUnits(2.38, Meters);
    public static final Distance minHeight = Distance.ofBaseUnits(0, Meters);

    // Trapezoidal constraints
    public static final double kMaxVelocity = 3;
    public static final double kMaxAcceleration = 3;

    // Conversion factors
    public static final double positionConversionFactor = 0.0354650904; // 0.1016/(9xpi)
    public static final double velocityConversionFactor = positionConversionFactor / 60;

    // Level heights (index is the level, begins at 0, which is ground)
    public static final Distance[] levelHeights = {
      Meters.of(0.000), // L0
      Meters.of(0.010), // L1
      Meters.of(0.500), // L2
      Meters.of(1.200), // L3
      Meters.of(2.390)  // L4
    };
    
    // Heights for misc. other tasks
    public static final Distance humanPlayerHeight = Meters.of(0.54);
    public static final Distance processorHeight = Meters.of(0.4);
    
    // PID & FF gains
    public static final double kS = 0.05701;
    public static final double kG = 0.99688;
    public static final double kV = 3.5529;
    public static final double kA = 1.0786;
    public static final double kP = 2.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
  }
}
