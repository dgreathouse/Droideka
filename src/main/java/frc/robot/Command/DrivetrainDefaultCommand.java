// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Subsystem.DrivetrainSubsystem;
import frc.robot.k.DRIVETRAIN;

public class DrivetrainDefaultCommand extends CommandBase {
    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(5);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(5);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(5);
  /** Creates a new DrivetrainDefaultCommand. */
  public DrivetrainDefaultCommand(DrivetrainSubsystem _subsystem) {
    addRequirements(_subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
     var xSpeed =
        -m_xspeedLimiter.calculate(MathUtil.applyDeadband(RobotContainer.m_XBOXDriver.getLeftY(), 0.2))
            * DRIVETRAIN.maxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
     var ySpeed =
        -m_yspeedLimiter.calculate(MathUtil.applyDeadband(RobotContainer.m_XBOXDriver.getLeftX(), 0.2))
            * DRIVETRAIN.maxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
     var rot =
        -m_rotLimiter.calculate(MathUtil.applyDeadband(RobotContainer.m_XBOXDriver.getRightX(), 0.2))
            * DRIVETRAIN.maxAngularSpeed;

            // xSpeed = -RobotContainer.m_XBOXDriver.getLeftY();
            // ySpeed = -RobotContainer.m_XBOXDriver.getLeftX();
            // rot = -RobotContainer.m_XBOXDriver.getRightX();

    RobotContainer.drivetrainSubsystem.drive(xSpeed, ySpeed, rot, false);
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
