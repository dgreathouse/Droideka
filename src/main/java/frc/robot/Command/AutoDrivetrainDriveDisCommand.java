// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AutoDrivetrainDriveDisCommand extends CommandBase {
  double maxSpeed = 0;
  double disInch = 0;
  double timeOut = 0;

  /** Creates a new AutoDrivetrainDriveDisCommand. */
  public AutoDrivetrainDriveDisCommand(double _maxSpeed, double _disInch, double _timeOut) {
    addRequirements(RobotContainer.drivetrainSubsystem);
    maxSpeed = _maxSpeed;
    disInch = _disInch;
    timeOut = _timeOut;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.drivetrainSubsystem.resetDriveEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.drivetrainSubsystem.driveAuto(maxSpeed, 0, 0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(RobotContainer.drivetrainSubsystem.getDriveDistanceInches() > disInch){
      RobotContainer.drivetrainSubsystem.driveAuto(0, 0, 0, true);
      return true;
    }
    return false;
  }
}
