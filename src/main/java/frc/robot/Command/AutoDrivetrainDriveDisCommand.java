// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.k;
import frc.robot.Subsystem.DrivetrainSubsystem;

public class AutoDrivetrainDriveDisCommand extends CommandBase {
  double maxSpeed = 0;
  double disMeters = 0;
  double timeOut = 0;
  double angle = 0;
  PIDController rotPIDController = new PIDController(.05, 0.01, 0);
  Timer driveTimer = new Timer();
  DrivetrainSubsystem drive;
  /** Creates a new AutoDrivetrainDriveDisCommand. */
  public AutoDrivetrainDriveDisCommand(double _maxSpeed, double _disInch, double _timeOut) {
    addRequirements(RobotContainer.drivetrainSubsystem);
    maxSpeed = _maxSpeed;
    disMeters = _disInch * k.DRIVETRAIN.MetersPerInch;
    timeOut = _timeOut;
    drive = RobotContainer.drivetrainSubsystem;

    rotPIDController.setTolerance(1);
    rotPIDController.setIntegratorRange(0, .5);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.drivetrainSubsystem.resetDriveEncoders();
    angle = RobotContainer.drivetrainSubsystem.getRobotAngle();
    driveTimer.reset();
    rotPIDController.reset();
    driveTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rot = rotPIDController.calculate(drive.getRobotAngle(),angle);
    drive.driveAuto(maxSpeed, 0, rot, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(RobotContainer.drivetrainSubsystem.getDriveDistanceMeters() > disMeters || driveTimer.hasElapsed(timeOut)){
      drive.drive(0, 0, 0, true, false);
      return true;
    }
    return false;
  }
}
