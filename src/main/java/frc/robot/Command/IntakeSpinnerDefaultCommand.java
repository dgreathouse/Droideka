// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
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
    double leftSpeed = -RobotContainer.operatorController.getLeftTriggerAxis() - RobotContainer.driverController.getLeftTriggerAxis();
    double rightSpeed = RobotContainer.operatorController.getRightTriggerAxis() + RobotContainer.driverController.getRightTriggerAxis();
    double speed = leftSpeed + rightSpeed;
    if(Math.abs(speed) < .25){
      speed = 0;
    }else if (Math.abs(speed) > .65){
      speed = 0.65 * Math.signum(speed);
    }
   // if(RobotContainer.operatorController.axisGreaterThan(2, 0.25).getAsBoolean() || RobotContainer.driverController.axisGreaterThan(2, 0.25).getAsBoolean()){
      RobotContainer.intake.spinHand(speed);
   // }else if(RobotContainer.operatorController.axisGreaterThan(3, 0.25).getAsBoolean() || RobotContainer.driverController.axisGreaterThan(3, 0.25).getAsBoolean()){
  //    RobotContainer.intake.spinHand(speed);
 //   }else {
 //     RobotContainer.intake.spinHand(0);
 //   }
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
