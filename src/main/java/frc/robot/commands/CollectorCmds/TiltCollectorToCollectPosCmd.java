// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CollectorCmds;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CollectorConstants;
import frc.robot.subsystems.Collector;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TiltCollectorToCollectPosCmd extends SequentialCommandGroup {
  Collector collector;

  /** Creates a new AlignCollectorToPickupNoteCmd. */
  public TiltCollectorToCollectPosCmd(Collector collector) {
    this.collector = collector;
    // Add your commands in thxe addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new TiltCollectorToSetpointCmd(collector, CollectorConstants.collectorTiltCollectPos));

  }
}
