// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.k;
import frc.robot.Lib.RotationMode;
import frc.robot.Lib.Util;
import frc.robot.Subsystem.DrivetrainSubsystem;
import frc.robot.k.DRIVETRAIN;

public class DrivetrainDefaultCommand extends CommandBase {
    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(5);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(5);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(5);

    PIDController m_rotationPIDController = new PIDController(k.DRIVETRAIN.rotKp, k.DRIVETRAIN.rotKi, k.DRIVETRAIN.rotKd);

  /** Creates a new DrivetrainDefaultCommand. */
  public DrivetrainDefaultCommand(DrivetrainSubsystem _subsystem) {
    addRequirements(_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_rotationPIDController.setTolerance(k.DRIVETRAIN.rotToleranceDeg, k.DRIVETRAIN.rotToleranceVel);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
     var xSpeed =
        -m_xspeedLimiter.calculate(MathUtil.applyDeadband(RobotContainer.driverController.getLeftY(), 0.2))
            * DRIVETRAIN.maxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
     var ySpeed =
        -m_yspeedLimiter.calculate(MathUtil.applyDeadband(RobotContainer.driverController.getLeftX(), 0.2))
            * DRIVETRAIN.maxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
     var rot =
        -m_rotLimiter.calculate(MathUtil.applyDeadband(RobotContainer.driverController.getRightX(), 0.2))
            * DRIVETRAIN.maxAngularSpeed;

    // If angle PID driving then set rotation to the PID value 
    double rotPID = m_rotationPIDController.calculate(RobotContainer.drivetrainSubsystem.getRobotAngle(),getRotationAngle());
    if(RobotContainer.drivetrainSubsystem.gRotationMode() == RotationMode.PIDAngle){
      rot = rotPID;
    }
    RobotContainer.drivetrainSubsystem.drive(xSpeed, ySpeed, rot, false);
  }
  /** Using the Right Thumb stick find the requested angle
   * 
   * @return the angle requested by the thumbstick
   */
  private double getRotationAngle(){
    double ang = Util.getAngle(RobotContainer.operatorController.getLeftX(), RobotContainer.operatorController.getLeftY());
    SmartDashboard.putNumber("RotationAngle", ang);
    return ang;
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
