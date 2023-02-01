// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Lib.ArmPosEnum;


public class AutoLeftConeCubeCommandGroup extends SequentialCommandGroup {
  /** Creates a new AutoLeftConeCube. */
  public AutoLeftConeCubeCommandGroup() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ArmSetCommand(ArmPosEnum.FAR_CONE),
      new ArmOnTargetCommand(),
      new ArmIntakeVelCommand(1),
      new WaitCommand(1),
      new ArmIntakeVelCommand(0),
      new ParallelCommandGroup(
        new ArmSetCommand(ArmPosEnum.HOME),
        new AutoDrivetrainPIDCommand(48, 5),
        new AutoDrivetrainRotatePIDCommand(20, 2)
      )
      /**
       * Place Cone on high level
       * Drive back to cube
       * Turn to cube
       * Pick up cube
       * Turn to scoring station
       * Drive to scoring station
       * Shift right to cube area
       * Drive forward to far cube 
       * drop cube.
       * Drive to charging station and balance
       */
    );
  }
}
