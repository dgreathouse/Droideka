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
public class AutoDrivetrainStrafePIDCommand extends PIDCommand {
  /** Creates a new AutoDrivetrainStrafePIDCommand. */
  public AutoDrivetrainStrafePIDCommand(double _distance, double timeOut) {
    super(
        // The controller that the command will use
        new PIDController(1, 0, 0),
        // This should return the measurement
        () -> RobotContainer.drivetrainSubsystem.getDriveDistance(),
        // This should return the setpoint (can also be a constant)
        () -> _distance,
        // This uses the output
        output -> {RobotContainer.drivetrainSubsystem.drive(0, output, 0, false);},
        RobotContainer.drivetrainSubsystem
        );
        
        m_controller.setTolerance(1);
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
