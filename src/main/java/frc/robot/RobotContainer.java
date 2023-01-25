// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Command.ArmDefaultCommand;
import frc.robot.Command.AutoLeftConeCubeCommandGroup;
import frc.robot.Command.DrivetrainDefaultCommand;
import frc.robot.Command.IntakeDefaultCommand;
import frc.robot.Subsystem.ArmSubsystem;
import frc.robot.Subsystem.DrivetrainSubsystem;
import frc.robot.Subsystem.IntakeSubsystem;

public class RobotContainer {
  public static DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private DrivetrainDefaultCommand m_drivetrainDefaultCommand = new DrivetrainDefaultCommand(drivetrainSubsystem);

  public static ArmSubsystem armSubsystem = new ArmSubsystem();
  private ArmDefaultCommand armDefaultCommand = new ArmDefaultCommand(armSubsystem);

  public static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private IntakeDefaultCommand intakeDefaultCommand = new IntakeDefaultCommand(intakeSubsystem);

  public static XboxController m_XBOXDriver = new XboxController(0);
  public static XboxController m_XBOXOperator = new XboxController(1);

  public RobotContainer() {
    drivetrainSubsystem.setDefaultCommand(m_drivetrainDefaultCommand);
    armSubsystem.setDefaultCommand(armDefaultCommand);
    intakeSubsystem.setDefaultCommand(intakeDefaultCommand);
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return new AutoLeftConeCubeCommandGroup();
  }
}
