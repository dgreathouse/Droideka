// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Lib.ArmPosEnum;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoLeftConeCubeCommandGroup extends SequentialCommandGroup {
  /** Creates a new AutoLeftConeCube. */
  public AutoLeftConeCubeCommandGroup() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ArmSetCommand(ArmPosEnum.FAR_CONE),
      new ArmIntakeVelCommand(1),
      new WaitCommand(1),
      new ArmIntakeVelCommand(0),
      new ArmSetCommand(ArmPosEnum.HOME)
      /**
       * Place Cone on high level
       * Drive back to cube
       * Turn to cube
       * Pick up cube
       * Turn to scoring station
       * Drive to scoring station
       * Shift right to cube area
       * Drive forward to far cone 
       * drop cube.
       * Drive to charging station and balance
       */
    );
  }
}
