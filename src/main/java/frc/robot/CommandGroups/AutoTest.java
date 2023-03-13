// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Command.ArmSetCommand;
import frc.robot.Command.AutoDrivetrainDrivePIDCommand;
import frc.robot.Command.AutoIntakeInCommand;
import frc.robot.Command.AutoIntakeOutCommand;
import frc.robot.Lib.ArmPosEnum;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoTest extends SequentialCommandGroup {
  /** Creates a new AutoTest. */
  public AutoTest() {
   // addRequirements(RobotContainer.drivetrainSubsystem, RobotContainer.intake, RobotContainer.arm);

    
    addCommands(
      new ArmSetCommand(ArmPosEnum.AUTO_FAR_CUBE),
      new WaitCommand(1),
      new AutoIntakeOutCommand(),
      new ArmSetCommand(ArmPosEnum.HOME),
      new WaitCommand(.5),
      new ParallelCommandGroup(
        new AutoIntakeInCommand(),
        new AutoDrivetrainDrivePIDCommand(1, 1, 0, 204, 10),
        new ArmSetCommand(ArmPosEnum.FLOOR_BACK_CUBE)
        
      ),
      new ArmSetCommand(ArmPosEnum.HOME)

      
    );
  }
}
