// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class IntakeOutSpinCommand extends CommandBase {
  boolean isFinished = false;
  double spinnerCnts = 0;
  /** Creates a new IntakeOutCommand. */
  public IntakeOutSpinCommand() {
    addRequirements(RobotContainer.intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
    spinnerCnts = RobotContainer.intake.getSpinnerCnts();
    //RobotContainer.intake.setSpinnerCnts(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.intake.getSpinnerCnts() > spinnerCnts + .3){
      isFinished = true;
    }
    RobotContainer.intake.spinHand(2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

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