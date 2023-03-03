// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Command.ArmIntakeVelCommand;
import frc.robot.Command.ArmSetCommand;
import frc.robot.Command.AutoDrivetrainDrivePIDCommand;
import frc.robot.Lib.ArmPosEnum;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBalance1CommandGroup extends SequentialCommandGroup {
  /** Creates a new AutoBalance1CommandGroup. */
  public AutoBalance1CommandGroup() {
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
      new AutoDrivetrainDrivePIDCommand(20,0, 0, 72, 4)
    );
  }
}
