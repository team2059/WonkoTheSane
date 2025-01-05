package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public final class Constants {

        public static final class DIOConstants {
                public static final int collectorOpticalDIO = 0;
                public static final int collectorTiltThruBoreDIO = 1;
                public static final int shooterTiltThruBoreEncoderDIO = 2;
                public static final int shooterOpticalDIO = 3;
                public static final int climberHallEffectDIO = 8;

                public static final int topShooterElevatorHallEffectDIO = 9;
                public static final int bottomShooterElevatorHallEffectDIO = 5;

        }

        public static final class PowerDistribution {
                public static final int pdpID = 15;
        }

        public static final class ShooterRegressionConstants {
                // Sample data points
                public static double[] distances = new double[] { 36, 56, 76, 96, 116, 136, 156 }; // inches
                public static double[] velocities = new double[] { 2000, 2500, 3000, 3250, 3500, 4000, 5000 };// RPMs
                // public static double[] angles = new double[] { -62.5, -50, -40, -35, -31,
                // -28, -25 }; // degrees
                public static double[] angles = new double[] { -64, -53, -43, -36.5, -32.5, -29.5, -26.5 }; // degrees

        }

        public static final class ShooterConstants {

                public static final double TILT_OFFSET = 0.659;

                public static final int shooterUpperID = 14;
                public static final int shooterLowerID = 13;
                public static final int indexerID = 16;
                public static final int shooterTiltID = 15;
                public static final int elevatorID = 10;

                public static final double TOP_LIMIT = 57.5;

                public static final double alignToCollectorPos = -50; // 0.78; // encoder value for getting note from
                                                                      // collector
                public static final double restPos = -90;// 0.91;

                public static final double tiltkP = 0;
                public static final double tiltkD = 0;
                public static final double elevatorkP = 0;
                public static final double elevatorkD = 0;

                // To measure Ks
                // manually, slowly increase the voltage to the mechanism until it starts to
                // move. The value of
                // is the largest voltage applied before the mechanism begins to move.

                // To tune Kv, increase the velocity feedforward gain
                // until the
                // flywheel approaches
                // the correct
                // setpoint over
                // time. If the
                // flywheel overshoots, reduce Kv.

                public static final SimpleMotorFeedforward shooterFF = new SimpleMotorFeedforward(0, 0, 0.0);
                public static final double shooterkP = 0.0;
                public static final double shooterkD = 0.0;
        }

        public static final class ClimberConstants {
                public static final int climberWinchMotorID = 9; // TODO: change placeholder CAN IDs

                public static final double climbkP = 0;
                public static final double climbkD = 0;

                public static final double COLLECTOR_POS_BEFORE_CLIMBING = 0.47;
                public static final double SHOOTER_TILT_TRAP_POS = 115;
        }

        public static final class CollectorConstants {

                public static final int collectorRollerID = 12;
                public static final int collectorTiltID = 11;

                public static final double collectorTiltAlignToShooterPos = 0.65; // encoder value for "retracted"
                                                                                  // collector
                public static final double collectorTiltCollectPos = 0.2; // encoder value for collecting

                public static final double tiltkD = 0.033;
                public static final double tiltkP = 1.33;

        }

        public static final class ScoringPresets {

                public static final double AMP_SHOOTER_HEIGHT = 57.5;
                public static final double AMP_SHOOTER_TILT = 35;
                public static final double AMP_SHOOTER_VELOCITY = 1250;

                public static final double SPEAKER_SUBWOOFER_SHOOTER_TILT = -60; // distance of 36 inches from the front
                                                                                 // of the bumper
                public static final double SPEAKER_SUBWOOFER_SHOOTER_VELOCITY = 2000; // distance of 36 inches from the
                                                                                      // front of the bumper

        }

        public static final class SwerveModuleConstants {

                // https://yagsl.gitbook.io/yagsl/configuring-yagsl/standard-conversion-factors

                public static final double rotationkP = 0.5;
                public static final double driveEncoderPositionConversionFactor = 0.047286787200699704;
                public static final double rotationEncoderPositionConversionFactor = 16.8 * 0.01745;

                public static final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0.2, 2.5, 0.0);
                public static final double drivekP = 0.0;

        }

        public static final class SwerveBaseConstants {

                /* Drivetrain Constants */
                public static final double trackWidth = Units.inchesToMeters(22.66);
                public static final double wheelBase = Units.inchesToMeters(24.5);

                // Max module speed, in m/s
                // Drive base radius in meters. Distance from robot center to furthest module.
                public static final double driveBaseRadius = Units.inchesToMeters(17.06);

                // nominal (real) divided by fudge factor
                public static final double wheelDiameter = Units.inchesToMeters(4.0 / 1.0);
                public static final double wheelCircumference = wheelDiameter * Math.PI;

                public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
                public static final double angleGearRatio = ((150.0 / 7.0) / 1.0); // 150/7:1

                public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                                new Translation2d(trackWidth / 2.0, wheelBase / 2.0), // front left
                                new Translation2d(trackWidth / 2.0, -wheelBase / 2.0), // front right
                                new Translation2d(-trackWidth / 2.0, wheelBase / 2.0), // rear left
                                new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0) // rear right
                );

                /* Swerve Profiling Values */

                public static final double maxSpeed = 4.5; // meters per second

                public static final double maxAngularVelocity = 11.5;

                public static final int frontLeftRotationMotorId = 6;
                public static final int frontLeftDriveMotorId = 5;

                public static final int frontRightRotationMotorId = 8;
                public static final int frontRightDriveMotorId = 7;

                public static final int rearLeftRotationMotorId = 2;
                public static final int rearLeftDriveMotorId = 1;

                public static final int rearRightRotationMotorId = 4;
                public static final int rearRightDriveMotorId = 3;

                public static final int frontLeftRotationEncoderId = 20;
                public static final int frontRightRotationEncoderId = 30;
                public static final int rearRightRotationEncoderId = 40;
                public static final int rearLeftRotationEncoderId = 50;

                public static final double frontLeftAngleOffset = Units.rotationsToRadians(0.992); // 20
                public static final double frontRightAngleOffset = Units.rotationsToRadians(0.143); // 30
                public static final double rearRightAngleOffset = Units.rotationsToRadians(0.917); // 40
                public static final double rearLeftAngleOffset = Units.rotationsToRadians(0.661); // 50

                // used
                public static final double translationkP = 3;
                public static final double rotationkP = 3;

                // Max linear velocity (M/S)
                public static final double maxVelocityMps = 3.0;

                // Max linear acceleration (M/S^2)
                public static final double maxAccelerationMpsSq = 3.0;

                // Max angular velocity (Rad/S)
                public static final double maxAngularVelocityRps = 2 * Math.PI;

                // Max angular acceleration (Rad/S^2)
                public static final double maxAngularAccelerationRpsSq = 4 * Math.PI;

                // max teleop speeds used in TeleopSwerve
                public static final double kTeleDriveMaxSpeedMetersPerSecond = 7.5 / 4.0;
                public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = 3.5;

        }

        public static final class VisionConstants {

                /**
                 * Standard deviations of model states. Increase these numbers to trust your
                 * model's state estimates less. This matrix is in the form [x, y, theta]ᵀ,
                 * with units in meters and radians, then meters.
                 */

                // values from Team Spectrum 3847’s X-Ray robot from last year
                // https://www.chiefdelphi.com/t/swerve-drive-pose-estimator-and-add-vision-measurement-using-limelight-is-very-jittery/453306/5
                public static final Vector<N3> STATE_STDS = VecBuilder.fill(0.1, 0.1, 10);

                /**
                 * Standard deviations of the vision measurements. Increase these numbers to
                 * trust global measurements from vision less. This matrix is in the form
                 * [x, y, theta]ᵀ, with units in meters and radians.
                 */

                // values from Team Spectrum 3847’s X-Ray robot from last year
                public static final Vector<N3> VISION_STDS = VecBuilder.fill(5, 5, 500);

                // Cam mounted facing forward, half a meter forward of center, half a meter up
                // from center.

                // you could always just instantiate it as a Transform3d() with no args or skip
                // it, and then the robot's origin and reference frame is centered on the camera

                // public static final Transform3d robotToCam = new Transform3d(
                // new Translation3d(Units.inchesToMeters(-15), 0.0,
                // Units.inchesToMeters(12.66)),
                // new Rotation3d(0, Units.degreesToRadians(-45), Units.degreesToRadians(180)));

                public static final Transform3d robotToCam = new Transform3d(
                                new Translation3d(Units.inchesToMeters(-13.25), 0.0,
                                                Units.inchesToMeters(12.66)),
                                new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(-36),
                                                Units.degreesToRadians(180)));

        }

        public static final class AutoConstants {

        }

}
