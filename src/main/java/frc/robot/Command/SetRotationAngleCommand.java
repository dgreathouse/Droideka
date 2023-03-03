// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class SetRotationAngleCommand extends InstantCommand {
  double angle = 0;
  /** Creates a new SetRotationAngleCommand. */
  public SetRotationAngleCommand(double _angle) {
    angle = _angle;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.drivetrainSubsystem.setRotationPIDAngle(angle);
  }


}
