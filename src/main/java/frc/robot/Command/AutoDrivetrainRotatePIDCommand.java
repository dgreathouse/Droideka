// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoDrivetrainRotatePIDCommand extends PIDCommand {
  /** Creates a new AutoDrivePIDCommand. */
  public AutoDrivetrainRotatePIDCommand(double _angle, double timeOut) {
    super(
        // The controller that the command will use
        new PIDController(1, 0, 0),
        // This should return the measurement
        () -> RobotContainer.drivetrainSubsystem.getRobotAngle(),
        // This should return the setpoint (can also be a constant)
        () -> Math.toRadians(_angle),
        // This uses the output
        output -> {
          RobotContainer.drivetrainSubsystem.drive(0, 0, output, false);
        });
        addRequirements(RobotContainer.drivetrainSubsystem);
        m_controller.setTolerance(1);
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(this.m_controller.atSetpoint()){
      return true;  
    }
    return false;
  }
}
