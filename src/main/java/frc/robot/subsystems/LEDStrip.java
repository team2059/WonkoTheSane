// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDStrip extends SubsystemBase {
  AddressableLED led;
  AddressableLEDBuffer buffer;

  public class RGB {
    public int r, g, b;
    public RGB(int r, int g, int b) {
      this.r = r;
      this.g = g;
      this.b = b;
    }
  }

  public RGB off = new RGB(0, 0, 0);
  public RGB white = new RGB(255, 255, 255);
  public RGB red = new RGB(255, 0, 0);
  public RGB green = new RGB(0, 255, 0);
  public RGB blue = new RGB(0, 0, 255);

  public Collector collector;
  public Shooter shooter;

  // team colors
  // TODO: find a good orange
  public RGB orange = new RGB(255, 255, 255);

  /** Creates a new LEDStrip. */
  public LEDStrip(Shooter shooter, Collector collector, int port, int length) {
    this.collector = collector;
    this.shooter = shooter;
    led = new AddressableLED(port);
    buffer = new AddressableLEDBuffer(length);

    led.setLength(buffer.getLength());
    setOff();
  }

  public void setRGB(RGB target) {
    for (int i = 0; i < buffer.getLength(); i++) {
      if (i%2==0) buffer.setRGB(i, target.r, target.g, target.b);
    }
    led.setData(buffer);
    led.start();
  }

  public void setOff() {
    setRGB(off);
  }

  public void waitXSeconds(double seconds) {
    try {
      Thread.sleep((long)(seconds * 1000));
    } catch (Exception e) {};
  }

  public void blinkSequence() {
    waitXSeconds(0.5);
    setRGB(off);
    waitXSeconds(0.5);
  }

  @Override
  public void periodic() {
      if (shooter.isNotePresent() && !collector.isNotePresent()) { // only shooter has note
        setRGB(red);
        //blinkSequence();
      } else if (collector.isNotePresent() && !shooter.isNotePresent()) { // only collector has note
        setRGB(blue);
        //blinkSequence();
      } else if (collector.isNotePresent() && shooter.isNotePresent()) { // both shooter & collector have note
        setRGB(green);
        //blinkSequence();
      } else { // none have a note
        setRGB(off);
      }
  }
}
