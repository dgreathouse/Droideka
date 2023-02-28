// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoDrivetrainDriveProPidCommand extends ProfiledPIDCommand {
  /** Creates a new AutoDrivetrainDriveProPidCommand. */
  public AutoDrivetrainDriveProPidCommand(double _distance, double _timeOut) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            0,
            0,
            0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(2, 0.5)),
        // This should return the measurement
        () -> RobotContainer.drivetrainSubsystem.getDriveDistanceMeters(),
        // This should return the goal (can also be a constant)
        () -> new TrapezoidProfile.State(),
        // This uses the output
        (output, setpoint) -> {
          // Use the output (and setpoint, if desired) here
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
