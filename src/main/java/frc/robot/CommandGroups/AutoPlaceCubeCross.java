// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Command.ArmSetCommand;
import frc.robot.Command.IntakeOutSpinCommand;
import frc.robot.Lib.ArmPosEnum;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPlaceCubeCross extends SequentialCommandGroup {
  public String getName(){
    return "Place Cube Cross";
  }
  /** Creates a new AutoPlaceCubeCross. */
  public AutoPlaceCubeCross() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ArmSetCommand(ArmPosEnum.FAR_CUBE),
      new WaitCommand(3),
      new IntakeOutSpinCommand(),
      new WaitCommand(1),
      
    );
  }
}
