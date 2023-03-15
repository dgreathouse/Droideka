// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.k;
import frc.robot.Subsystem.DrivetrainSubsystem;
import frc.robot.k.DRIVETRAIN;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoDrivetrainDrivePIDCommand extends CommandBase {
  PIDController rotPIDController = new PIDController(.05, 0.01, 0);
  ProfiledPIDController drivePIDController = new ProfiledPIDController(3, 4, 0, new TrapezoidProfile.Constraints(2,1.1));
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
  double maxSpeed = 0;
  DrivetrainSubsystem drive;
  /** Creates a new AutoDrivePIDCommand. */
  /**
   * 
   * @param _maxSpeed The Speed in MPS 4.36 is the max speed
   * @param _x X Direction +/-1 
   * @param _y Y Direction +/-1
   * @param _distance Distance in inches
   * @param _timeOut Time to stop trying to reach goal
   */
  public AutoDrivetrainDrivePIDCommand(double _maxSpeed, double _x, double _y, double _distance, double _timeOut) {
    addRequirements(RobotContainer.drivetrainSubsystem);
    drive = RobotContainer.drivetrainSubsystem;
    distance = _distance * k.DRIVETRAIN.MetersPerInch; // Convert to meters
    driveTimeOut = _timeOut;
    x = _x * DRIVETRAIN.maxSpeed;
    y = _y * DRIVETRAIN.maxSpeed;
    maxSpeed = _maxSpeed;
    // Drive
    drivePIDController.setTolerance(.01,.5);
    drivePIDController.setIntegratorRange(-3, 3);
    

    // Rotation
    rotPIDController.setTolerance(1);
    rotPIDController.setIntegratorRange(0, .5);

    drivePIDController.reset(0);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angle = RobotContainer.drivetrainSubsystem.getRobotAngle();
    RobotContainer.drivetrainSubsystem.resetDriveEncoders();
    steerTimer.reset();
    driveTimer.reset();
    rotPIDController.reset();
    steerTimer.start();
    
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
    // Steer
    if(!steerTimer.hasElapsed(steerTime)){
      //SmartDashboard.putNumber("Test", cnt++);
      drive.steerAuto(x,y);
    }else{

      if(!resetDone){
        drive.resetDriveEncoders();
        resetDone = true;
        driveTimer.start();
        
        
      }else {    // Drive
        double drv = drivePIDController.calculate(drive.getDriveDistanceMeters(),distance);
       
        double rot = rotPIDController.calculate(drive.getRobotAngle(),angle);
        
        //drv = Util.limit(drv, -maxSpeed, maxSpeed);
        drive.driveAuto(drv, 0, rot,true);
      }
    }
    SmartDashboard.putBoolean("AtSetpoint",drivePIDController.atSetpoint());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if(drivePIDController.atGoal() || driveTimer.hasElapsed(driveTimeOut)){
    
    SmartDashboard.putNumber("DriveTime", driveTimer.get());
      drive.drive(0, 0, 0, false, false);
      return true;
    }else {
      return false;
    }
  }
}
