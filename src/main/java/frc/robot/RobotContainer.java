// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Command.ArmDefaultCommand;
import frc.robot.Command.ArmSetCommand;
import frc.robot.Command.AutoDoNothingCommandGroup;
import frc.robot.Command.AutoTest;
import frc.robot.Command.DrivetrainDefaultCommand;
import frc.robot.Command.SetRotationAngleCommand;
import frc.robot.Command.SwitchFieldDriveMode;
import frc.robot.Command.SwitchGyroCommand;
import frc.robot.Command.SwitchRotationMode;
import frc.robot.CommandGroups.AutoLeftConeCubeCommandGroup;
import frc.robot.Lib.ArmPosEnum;
import frc.robot.Subsystem.ArmSubsystem;
import frc.robot.Subsystem.DrivetrainSubsystem;



public class RobotContainer {
  public static DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private DrivetrainDefaultCommand drivetrainDefaultCommand = new DrivetrainDefaultCommand(drivetrainSubsystem);

  public static ArmSubsystem armSubsystem = new ArmSubsystem();
  private ArmDefaultCommand armDefaultCommand = new ArmDefaultCommand(armSubsystem);

  public static CommandXboxController driverController = new CommandXboxController(0);
 // public static CommandXboxController operatorController = new CommandXboxController(1);
  SendableChooser<Command> autoChooser = new SendableChooser<>();

  /** RobotContainer holds all the static data for references to the subsystems.
   * To call a method in the subsystem use the following code example
   * RobotContainer.armSubsystem.EnterMethodName();
   */
  public RobotContainer() {
    drivetrainSubsystem.setDefaultCommand(drivetrainDefaultCommand);
    armSubsystem.setDefaultCommand(armDefaultCommand);

    configureBindings();

    autoChooser.addOption("Left Cone, Get Cube, Balance", new AutoLeftConeCubeCommandGroup());
    autoChooser.addOption("Test", new AutoTest());
    autoChooser.setDefaultOption("Do Nothing", new AutoDoNothingCommandGroup());
    
    // Add more auto options here 
    SmartDashboard.putData(autoChooser);

  }
  /** Configure the XBOX controller bindings from buttons/axis to Commands */
  private void configureBindings() {
    // Driver Controller bindings
    driverController.start().onTrue(new SwitchGyroCommand());
    driverController.x().onTrue(new SwitchRotationMode());
    driverController.b().onTrue(new SwitchFieldDriveMode());
    driverController.a().onTrue(new SetRotationAngleCommand(0));
    driverController.y().onTrue(new SetRotationAngleCommand(180));

    // Operator Controller Bindings
    // operatorController.leftBumper().and(operatorController.rightBumper()).onTrue(new ArmSetCommand(ArmPosEnum.HOME));
    // operatorController.leftBumper().and(operatorController.x()).onTrue(new ArmSetCommand(ArmPosEnum.WALL_CONE));
    // operatorController.leftBumper().and(operatorController.y()).onTrue(new ArmSetCommand(ArmPosEnum.FAR_CONE));
    // operatorController.leftBumper().and(operatorController.b()).onTrue(new ArmSetCommand(ArmPosEnum.MID_CONE));
    // operatorController.leftBumper().and(operatorController.a()).onTrue(new ArmSetCommand(ArmPosEnum.LOW_CONE));
    // operatorController.rightBumper().and(operatorController.x()).onTrue(new ArmSetCommand(ArmPosEnum.WALL_CUBE));
    // operatorController.rightBumper().and(operatorController.y()).onTrue(new ArmSetCommand(ArmPosEnum.FAR_CUBE));
    // operatorController.rightBumper().and(operatorController.b()).onTrue(new ArmSetCommand(ArmPosEnum.MID_CUBE));
    // operatorController.rightBumper().and(operatorController.a()).onTrue(new ArmSetCommand(ArmPosEnum.LOW_CUBE));
    // operatorController.start().onTrue(new ArmSetCommand(ArmPosEnum.FLOOR_FRONT_CUBE));

  }
  /** Return the selected command from the smartdashboard on the drivestation */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
