// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command;

import java.io.Console;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoDrivetrainDrivePIDCommand extends CommandBase {
  PIDController rotPIDController = new PIDController(2.75, .5, 0);
  PIDController drivePIDController = new PIDController(1, .1, 0);
  double distance = 0;
  double driveTimeOut = 0;
  double angle = 0;
  double steerTime = 1;
  Timer steerTimer = new Timer();
  Timer driveTimer = new Timer();
  double x = 0;
  double y = 0;
  boolean resetDone = false;
  int cnt = 0;
  /** Creates a new AutoDrivePIDCommand. */
  public AutoDrivetrainDrivePIDCommand(double _x, double _y, double _distance, double _timeOut) {
    addRequirements(RobotContainer.drivetrainSubsystem);
    distance = _distance;
    driveTimeOut = _timeOut;
    x = _x;
    y = _y;
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
    steerTimer.reset();
    driveTimer.reset();
    rotPIDController.reset();
    drivePIDController.reset();
    steerTimer.start();
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
    // Steer
    if(!steerTimer.hasElapsed(steerTime)){
      SmartDashboard.putNumber("Test", cnt++);
      RobotContainer.drivetrainSubsystem.steerAuto(x,y);
    }else{

      if(!resetDone){
        RobotContainer.drivetrainSubsystem.resetDriveEncoders();
        resetDone = true;
        driveTimer.start();
      }else {
    // Drive
        double drv = drivePIDController.calculate(RobotContainer.drivetrainSubsystem.getDriveDistanceInches(),distance);
        double rot = rotPIDController.calculate(RobotContainer.drivetrainSubsystem.getRobotAngle(),angle);
        RobotContainer.drivetrainSubsystem.driveAuto(drv, 0, rot,true);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(drivePIDController.atSetpoint() || driveTimer.hasElapsed(driveTimeOut)){
      return true;
    }
    return false;
  }
}
