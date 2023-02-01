// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Command.ArmDefaultCommand;
import frc.robot.Command.ArmSetCommand;
import frc.robot.Command.AutoDoNothingCommandGroup;
import frc.robot.Command.AutoLeftConeCubeCommandGroup;
import frc.robot.Command.DrivetrainDefaultCommand;

import frc.robot.Command.SwitchGyroCommand;
import frc.robot.Lib.ArmPosEnum;
import frc.robot.Subsystem.ArmSubsystem;
import frc.robot.Subsystem.DrivetrainSubsystem;



public class RobotContainer {
  public static DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private DrivetrainDefaultCommand m_drivetrainDefaultCommand = new DrivetrainDefaultCommand(drivetrainSubsystem);

  public static ArmSubsystem armSubsystem = new ArmSubsystem();
  private ArmDefaultCommand armDefaultCommand = new ArmDefaultCommand(armSubsystem);


  public static XboxController m_XBOXDriver = new XboxController(0);
  public static XboxController m_XBOXOperator = new XboxController(1);

  SendableChooser<Command> autoChooser = new SendableChooser<>();


  public RobotContainer() {
    drivetrainSubsystem.setDefaultCommand(m_drivetrainDefaultCommand);
    armSubsystem.setDefaultCommand(armDefaultCommand);

    configureBindings();

    autoChooser.addOption("Left Cone, Get Cube, Balance", new AutoLeftConeCubeCommandGroup());
    autoChooser.setDefaultOption("Do Nothing", new AutoDoNothingCommandGroup());
    // Add more auto options here
    
    SmartDashboard.putData(autoChooser);



  }

  private void configureBindings() {
    CommandXboxController operatorController = new CommandXboxController(1);
    Trigger switchGyroTrigger = new JoystickButton(m_XBOXDriver, XboxController.Button.kStart.value);
    switchGyroTrigger.onTrue(new SwitchGyroCommand());

    // Arm Bindings
    operatorController.leftBumper().and(operatorController.rightBumper()).onTrue(new ArmSetCommand(ArmPosEnum.HOME));
    operatorController.leftBumper().and(operatorController.a()).onTrue(new ArmSetCommand(ArmPosEnum.WALL_CONE));
 

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
