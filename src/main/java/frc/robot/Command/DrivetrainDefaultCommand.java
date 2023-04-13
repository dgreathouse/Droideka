// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.k;
import frc.robot.Subsystem.DrivetrainSubsystem;
import frc.robot.k.DRIVETRAIN;

public class DrivetrainDefaultCommand extends CommandBase {
    // Slew rate limiters to make joystick inputs more gentle; 1/2 sec from 0 to 1.
    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(1);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(1);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(1);

    PIDController m_rotationPIDController = new PIDController(k.DRIVETRAIN.rotKp, k.DRIVETRAIN.rotKi, k.DRIVETRAIN.rotKd);
    DrivetrainSubsystem drive;
  /** Creates a new DrivetrainDefaultCommand. */
  public DrivetrainDefaultCommand(DrivetrainSubsystem _subsystem) {
    addRequirements(_subsystem);
    drive = _subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_rotationPIDController.setTolerance(k.DRIVETRAIN.rotToleranceDeg, k.DRIVETRAIN.rotToleranceVel);
    m_rotationPIDController.setIntegratorRange(0, .5);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

     var xSpeed = -m_xspeedLimiter.calculate(MathUtil.applyDeadband(RobotContainer.driverController.getLeftY(), k.DRIVETRAIN.stickDeadband))
            * DRIVETRAIN.maxSpeed;

     var ySpeed = -m_yspeedLimiter.calculate(MathUtil.applyDeadband(RobotContainer.driverController.getLeftX(), k.DRIVETRAIN.stickDeadband))
            * DRIVETRAIN.maxSpeed;

     var rot = -m_rotLimiter.calculate(MathUtil.applyDeadband(RobotContainer.driverController.getRightX(), k.DRIVETRAIN.stickDeadband))
            * DRIVETRAIN.maxAngularSpeed;


    RobotContainer.drivetrainSubsystem.drive(xSpeed * k.DRIVETRAIN.speedScale, ySpeed * k.DRIVETRAIN.speedScale, rot*k.DRIVETRAIN.rotationScale,RobotContainer.drivetrainSubsystem.isFieldRelative,true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
