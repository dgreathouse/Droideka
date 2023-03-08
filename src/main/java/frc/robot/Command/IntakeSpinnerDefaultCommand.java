// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class IntakeSpinnerDefaultCommand extends CommandBase {
  /** Creates a new IntakeSpinnerDefaultCommand. */
  public IntakeSpinnerDefaultCommand(Subsystem _sys) {
    addRequirements(_sys);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.operatorController.axisGreaterThan(2, 0.25).getAsBoolean()){
      RobotContainer.intake.spinHand(-RobotContainer.operatorController.getLeftTriggerAxis());
    }else if(RobotContainer.operatorController.axisGreaterThan(3, 0.25).getAsBoolean()){
      RobotContainer.intake.spinHand(RobotContainer.operatorController.getRightTriggerAxis());
    }else {
      RobotContainer.intake.spinHand(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
