// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.RobotContainer;
import frc.robot.Lib.ArmPosEnum;


public class ArmSetCommand extends InstantCommand {
  ArmPosEnum m_pos;
  /** 
 * Set the requested position for the arm to move to. 
 * This is an InstantCommand which means it will run once and exit
 */
  public ArmSetCommand(ArmPosEnum _pos) {
    addRequirements(RobotContainer.armSubsystem);
    m_pos = _pos;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   // RobotContainer.armSubsystem.m_armPos = m_pos;
   // RobotContainer.armSubsystem.resetMoveTimers();
  }
}
