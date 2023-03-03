// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Lib.Util;
import frc.robot.Lib.Util.Direction;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoGamePieceCommandGroup extends SequentialCommandGroup {
  Util.Direction dir = Direction.LEFT;
  /** Creates a new AutoGamePieceCommandGroup. */
  public AutoGamePieceCommandGroup(Util.Direction _dir) {
    dir = _dir;
    double sign = 1.0;
    if(dir == Util.Direction.RIGHT){
      sign = -1.0;
    }
    /**
     * Score game piece
     * Get game piece from field
     * return to scoring area and score game piece
     * Drive to pickup other game piece in field.
     */
    addCommands(

    );
  }
}
