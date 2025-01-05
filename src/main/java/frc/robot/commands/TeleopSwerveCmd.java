package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveBaseConstants;
import frc.robot.subsystems.SwerveBase;

public class TeleopSwerveCmd extends Command {

  /**
   * Command to allow for driver input in teleop
   * Can't be inlined efficiently if we want to edit the inputs in any way
   * (deadband, square, etc.)
   */

  private final SwerveBase drive;

  /**
   * Joysticks return DoubleSuppliers when the get methods are called
   * This is so that joystick getter methods can be passed in as a parameter but
   * will continuously update,
   * versus using a double which would only update when the constructor is called
   */
  private final DoubleSupplier forwardX;
  private final DoubleSupplier forwardY;
  private final DoubleSupplier rotation;
  private final DoubleSupplier slowSlider;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
  private final Supplier<Boolean> fieldOrientedFunction;
  private final Supplier<Boolean> inverted;
  private final Supplier<Boolean> strafeOnly;
  private final Supplier<Boolean> slowAll;

  public TeleopSwerveCmd(
      SwerveBase subsystem,
      DoubleSupplier fwdX,
      DoubleSupplier fwdY,
      DoubleSupplier rot,
      DoubleSupplier slowSlider,
      Supplier<Boolean> fieldOrientedFunction, Supplier<Boolean> inverted,
      Supplier<Boolean> strafeOnly,
      Supplier<Boolean> slowAll) {

    drive = subsystem;
    forwardX = fwdX;
    forwardY = fwdY;
    rotation = rot;
    this.slowSlider = slowSlider;

    this.fieldOrientedFunction = fieldOrientedFunction;
    this.inverted = inverted;
    this.strafeOnly = strafeOnly;
    this.slowAll = slowAll;

    this.xLimiter = new SlewRateLimiter(SwerveBaseConstants.maxAccelerationMpsSq);
    this.yLimiter = new SlewRateLimiter(SwerveBaseConstants.maxAccelerationMpsSq);
    this.turningLimiter = new SlewRateLimiter(SwerveBaseConstants.maxAccelerationMpsSq);

    addRequirements(subsystem);

  }

  @Override
  public void execute() {

    /**
     * Units are given in meters per second radians per second
     * Since joysticks give output from -1 to 1, we multiply the outputs by the max
     * speed
     * Otherwise, our max speed would be 1 meter per second and 1 radian per second
     */

    double fwdX = forwardX.getAsDouble();
    double fwdY = forwardY.getAsDouble();
    double rot = rotation.getAsDouble();

    // 2. Apply deadband
    fwdX = Math.abs(fwdX) > 0.25 ? fwdX : 0.0;
    fwdY = Math.abs(fwdY) > 0.33 ? fwdY : 0.0;
    rot = Math.abs(rot) > 0.4 ? rot : 0.0;

    // 3. Make the driving smoother
    fwdX = xLimiter.calculate(fwdX) * SwerveBaseConstants.kTeleDriveMaxSpeedMetersPerSecond;
    fwdY = yLimiter.calculate(fwdY) * SwerveBaseConstants.kTeleDriveMaxSpeedMetersPerSecond;
    rot = turningLimiter.calculate(rot)
        * SwerveBaseConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

    double slowVal = 0.25 + ((Math.abs(slowSlider.getAsDouble() - 1)) / (2.0) * 0.75);

    // System.out.println("slow val = " + slowVal);

    if (inverted.get() == true) {
      fwdX *= -1;
      fwdY *= -1;
      rot *= 1;
    } else {
      fwdX *= 1;
      fwdY *= 1;
      rot *= 1;
    }

    if (slowAll.get() == true) {
      drive.drive(
          fwdX *= slowVal * 0.5,
          fwdY *= slowVal * 0.5,
          rot *= slowVal * 0.5,
          fieldOrientedFunction.get());
    }

    if (strafeOnly.get() == true) {

      fwdX *= 2 * slowVal * 0.66;
      fwdY *= 2 * slowVal * 0.66;
      rot = 0;

    } else {
      fwdX *= 2.25 * slowVal;
      fwdY *= 2.25 * slowVal;
      rot *= 1.5 * slowVal;

    }

    drive.drive(
        -MathUtil.applyDeadband(fwdX, 0.1, 0.75),
        -MathUtil.applyDeadband(fwdY, 0.3, 0.75),
        -MathUtil.applyDeadband(rot * 0.5, 0.3, 0.75),
        fieldOrientedFunction.get());

  }

}