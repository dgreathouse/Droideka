// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Command.ArmIntakeVelCommand;
import frc.robot.Command.ArmOnTargetCommand;
import frc.robot.Command.ArmSetCommand;
import frc.robot.Command.AutoDrivetrainBalancePIDCommand;
import frc.robot.Command.AutoDrivetrainDrivePIDCommand;
import frc.robot.Command.AutoDrivetrainStrafePIDCommand;
import frc.robot.Lib.ArmPosEnum;


public class AutoLeftConeCubeCommandGroup extends SequentialCommandGroup {
  /** Creates a new AutoLeftConeCube. */
  public AutoLeftConeCubeCommandGroup() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
       new ArmSetCommand(ArmPosEnum.FAR_CONE),
       new WaitCommand(1),
       new ArmIntakeVelCommand(1),
       new WaitCommand(1),
       new ArmIntakeVelCommand(0),
       new ArmSetCommand(ArmPosEnum.HOME),
       new WaitCommand(1),
       new AutoDrivetrainStrafePIDCommand(-7, 0),
       new AutoDrivetrainDrivePIDCommand(1,0,1,-7, 2)


    );
  }
}
