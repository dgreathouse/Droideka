  // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Command.DrivetrainDefaultCommand;

import frc.robot.Command.SwitchDriveNeutralMode;
import frc.robot.Command.SwitchFieldDriveMode;
import frc.robot.Command.SwitchGyroCommand;

import frc.robot.Subsystem.DrivetrainSubsystem;



public class RobotContainer {
  public static DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private DrivetrainDefaultCommand drivetrainDefaultCommand = new DrivetrainDefaultCommand(drivetrainSubsystem);



  public static CommandXboxController driverController = new CommandXboxController(0);
  public static CommandXboxController operatorController = new CommandXboxController(1);
  
  SendableChooser<Command> autoChooser = new SendableChooser<>();

  /** RobotContainer holds all the static data for references to the subsystems.
   * To call a method in the subsystem use the following code example
   * RobotContainer.armSubsystem.EnterMethodName();
   */
  public RobotContainer() {
    drivetrainSubsystem.setDefaultCommand(drivetrainDefaultCommand);

    configureBindings();


   // autoChooser.addOption("Test", new AutoTest());

    
    // Add more auto options here 


    //SmartDashboard.putData(drivetrainSubsystem);
    LiveWindow.enableAllTelemetry();

  }
  /** Configure the XBOX controller bindings from buttons/axis to Commands */
  private void configureBindings() {
    /*************************** Driver Control Buttons  *********************************************************/
    driverController.x().onTrue(new SwitchGyroCommand());
    driverController.a().onTrue(new SwitchDriveNeutralMode());
    driverController.b().onTrue(new SwitchFieldDriveMode());

    /**************************************** HOME Command *******************************************************/


    /***************** Operator Control Buttons (CONES) **********************************************************/


    /************************* Operator Control Buttons (CUBES) **************************************************/

    drivetrainSubsystem.resetSteerEncoders();
    
   // LiveWindow.disableAllTelemetry();

  }
  /** Return the selected command from the smartdashboard on the drivestation */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
