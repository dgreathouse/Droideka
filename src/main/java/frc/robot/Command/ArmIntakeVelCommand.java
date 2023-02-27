// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmIntakeVelCommand extends InstantCommand {
  double m_vel = 0;
  /**
   * Create a new ArmIntakeVelCommand(double velocity)
   * Sets the velocity of the intake from +/- 1 
   * This command will exit immediatly 
   * @param _vel The Velocity from +/- 1
   */
  public ArmIntakeVelCommand(double _vel) {
    //addRequirements(RobotContainer.arm);
    m_vel = _vel;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.arm.spinHand(m_vel);
  }
}
