# WonkoTheSane

Team 2059 "The Hitchhikers" 2025 *Reefscape* code, written in Java.

## Hardware

Drivetrain
- 4x SDS MK4n L2+ swerve modules with NEO Vortex & billet wheels
- 4x CTRE CANcoder absolute encoders for each swerve module
- NavX2

## Notes

### Basic Vision Tuning

1. "For all pipelines, exposure time should be set as low as possible while still allowing for the target to be reliably tracked. This allows for faster processing as decreasing exposure will increase your camera FPS."
2. "Unlike with retroreflective tape, AprilTag tracking is not very dependent on lighting consistency. If you have trouble detecting tags due to low light, you may want to try increasing exposure, but this will likely decrease your achievable framerate."
3. "Cranking your exposure as low as it goes and increasing your gain/brightness. This will decrease the effects of motion blur and increase FPS."

### Clearing RoboRIO deploy directory

Instructions from previous years are obsolete, the `deleteOldFiles` flag in [build.gradle](build.gradle) automatically updates the folder as necessary upon redeploy.

### Finding CANcoder Offsets

For finding the offsets, use a piece of 1x1 metal that is straight against the forks of the front and back modules (on the left and right side) to ensure that the modules are straight.

Preferably you should have the wheels facing in the direction where a positive input to the drive motor drives forward. If for some reason you set the offsets with the wheels backwards, you can change motor inversions to fix.

Open Phoenix Tuner X and get the *absolute* position reading from the perfectly straightened wheels.

### Setting CAN IDs

All REV products can have IDs and preferences set via [REV Hardware Client](https://docs.revrobotics.com/rev-hardware-client).

CTRE products, like CANcoders can be configured via [Phoenix Tuner X](https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/index.html).

Make sure that:
- No two devices have the same ID.
- All devices on the network have the latest firmware and have been reset to factory defaults.

### PID tuning

1. Start with a low P value (such as 0.01, maybe even lower).
2. Multiply by 10 until oscillating around the setpoint
3. Scale back by searching for the value (for example, if it starts oscillating at a P of 10, then try (10 -> 5 -> 7.5 -> etc)) until the module overshoots the setpoint but corrects with no oscillation.
4. Repeat the process for D. The D value will basically help prevent the overshoot. Ignore I unless absolutely necessary

### System Identification

This is the process of determining a mathematical model for the predicted outcome of a system through analysis of its inputs and outputs. In this case, how input voltage affects measurements, such as encoder data.

#### Identification routines

Two types of tests, each run forward & backwards for 4 total tests

1. Quasistatic: mechanism is gradually sped up such that voltage relating to acceleration is negligible

2. Dynamic: "step voltage" is applied to the system, so that behavior while accelerating can be determined

For these routines, WPIlib **SysID** is used. In past seasons, **SysID** would create and deploy a sample project in order to gather data, but this year you create `SysIDRoutines` yourself, and feed the data from that routine into **SysID** to calculate your constants.

### Swerve programming resources

See [this document](https://docs.google.com/document/d/1VO_HjHx0AQW0lgfmrxxUV-ULtsHZ8Dktv3-T3pGWbyM/edit?tab=t.0) for swerve resources. Some library usage could be outdated