// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoDrivetrainDrivePIDCommand extends CommandBase {
  PIDController rotPIDController = new PIDController(2.75, .5, 0);
  PIDController drivePIDController = new PIDController(1, .1, 0);
  double distance = 0;
  double timeOut = 0;
  double angle = 0;
  /** Creates a new AutoDrivePIDCommand. */
  public AutoDrivetrainDrivePIDCommand(double _distance, double _timeOut) {
    distance = _distance;
    timeOut = _timeOut;
    // Drive
    drivePIDController.setTolerance(1);
    drivePIDController.setIntegratorRange(0, 1);

    // Rotation
    rotPIDController.setTolerance(1);
    rotPIDController.setIntegratorRange(0, .5);


    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angle = RobotContainer.drivetrainSubsystem.getRobotAngle();
    RobotContainer.drivetrainSubsystem.resetDriveEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Drive
    double drv = drivePIDController.calculate(RobotContainer.drivetrainSubsystem.getDriveDistanceInches(),distance);
    double rot = rotPIDController.calculate(RobotContainer.drivetrainSubsystem.getRobotAngle(),angle);
    RobotContainer.drivetrainSubsystem.drive(drv, 0, rot,false,true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drivePIDController.atSetpoint();
  }
}
