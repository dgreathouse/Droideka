// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class IntakeInSpinCommand extends CommandBase {

  boolean isFinished = false;
  double spinnerCnts = 0;
  /** Creates a new IntakeSpinCommand. */
  public IntakeInSpinCommand() {  
    addRequirements(RobotContainer.intake);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
    //RobotContainer.intake.setSpinnerCnts(0);
    //SmartDashboard.putNumber("SpinnerInitCnts", RobotContainer.intake.getSpinnerCnts());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // if(RobotContainer.intake.getSpinnerCnts() < -.1){
    //   isFinished = true;
    // }
    RobotContainer.intake.spinHand(-0.5);
  }

  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(isFinished){
      RobotContainer.intake.spinHand(0.0);
      return true;
    }
    return false;
  }
}
