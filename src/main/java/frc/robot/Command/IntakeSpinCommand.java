// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Lib.Util.Direction;

public class IntakeSpinCommand extends CommandBase {
  Direction dir = Direction.OFF;
  Timer timer = new Timer();
  double amps = 0;
  boolean isFinished = false;
  /** Creates a new IntakeSpinCommand. */
  public IntakeSpinCommand(Direction _dir) {  
    dir = _dir;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString("dir", dir.toString());
    amps = RobotContainer.arm.getIntakeCurrent();
    if(dir == Direction.IN){
      RobotContainer.arm.spinHand(0.5);
      // if(amps > 5 || timer.hasElapsed(1)){
      //   isFinished = true;
      // }
    }else if(dir == Direction.OUT){
      RobotContainer.arm.spinHand(-0.5);
      // if(timer.hasElapsed(1)){
      //   isFinished = true;
      // }
    }else if(dir == Direction.OFF){
     // isFinished = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(isFinished){
      RobotContainer.arm.spinHand(0.0);
    }
    return isFinished;
  }
}
