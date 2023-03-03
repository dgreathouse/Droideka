// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.RobotContainer;


public class AutoDrivetrainBalancePIDCommand extends PIDCommand {
  /** Creates a new AutoDrivetrainBalancePIDCommand. */
  public AutoDrivetrainBalancePIDCommand() {
    super(
        // The controller that the command will use
        new PIDController(1, 0, 0),
        // This should return the measurement
        () -> RobotContainer.drivetrainSubsystem.getRobotPitch(),
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          RobotContainer.drivetrainSubsystem.drive(output, 0, 0, false,false);
        },
        RobotContainer.drivetrainSubsystem
        );
    m_controller.setTolerance(0.1);
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_controller.atSetpoint()){
      return true;
    }
    return false;
  }
}
