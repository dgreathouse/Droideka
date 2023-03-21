// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AutoDriveBalancePIDCommand extends CommandBase {
  /** Drive until the gyro pitch says +/- 5 */
  public AutoDriveBalancePIDCommand() {
    addRequirements(RobotContainer.drivetrainSubsystem);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.drivetrainSubsystem.driveAuto(1, 0, 0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(RobotContainer.drivetrainSubsystem.getRobotPitch() > -10 && RobotContainer.drivetrainSubsystem.getRobotPitch() < 10){
      RobotContainer.drivetrainSubsystem.driveAuto(0, 0, 0, true);
      return true;
    }
    return false;
  }
}
