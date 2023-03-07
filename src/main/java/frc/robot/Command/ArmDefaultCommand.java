// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Lib.ArmPosEnum;
import frc.robot.Subsystem.Arm;


public class ArmDefaultCommand extends CommandBase {
  Arm arm;
  double bicepPos = 0;
  double elbowPos = 0;
  double intakePos = 0;
  double intakeVel = 0;
double intakeCurrentLim = 0;
double intakeTimeLimit = 1;
  /** Creates a new ArmDefaultCommand. */
  public ArmDefaultCommand(Arm _subsystem) {
    addRequirements(_subsystem);
    arm = _subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  /**
     * What must be done here;
     * Move to positions and set intake at a speed
   */
  @Override
  public void execute() {
      
      // Move to position. Buttons change enum in arm
      arm.m_armController.moveToPosition();
      
      // // Get the Left and Right Trigger Axis values
      double lAxis = RobotContainer.operatorController.getLeftTriggerAxis();
      double rAxis = RobotContainer.operatorController.getRightTriggerAxis();
      double speed = lAxis - rAxis;
     // Default speed = 0;
      arm.spinHand(speed);
      SmartDashboard.putNumber("Intake Current", arm.getIntakeCurrent());
      // // Flags for direction
      // boolean in = false;
      // boolean out = false;
      // boolean off = true;
      // // Set flags based on the trigger being pulled in.
      // in = lAxis > 0.5 ? true : false;
      // out = rAxis > 0.5 ? true : false;
      // // Filter the in-out-off
      // if(in == true && out == true){
      //   in = false; out = false; off = true;
      // }else if(in == true && out == false){
      //   off = false;
      // }else if(in == false && out == true){
      //   off = false;
      // }else if(in == false && out == false){
      //   off = true;
      // }

      // if(off){
      //   speed = 0;
      // }
      
      // // Spin the intake based on the L/R trigger
      // ArmPosEnum pos = arm.m_armController.m_armPos;
      // switch(pos){
      //   case HOME:
      //   case WALL_CONE:
        
      //   case WALL_CUBE:
      //   case FAR_CONE:
      //   case FAR_CUBE:
      //   case MID_CONE:
      //   case MID_CUBE:
      //   case LOW_CONE:
      //   case LOW_CUBE:
      //   case FLOOR_BACK_CUBE:
      //   default:



      // }
      // RobotContainer.arm.spinHand(speed);
   }
 
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
