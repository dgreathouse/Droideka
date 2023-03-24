// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;

public class IntakeSpinnerDefaultCommand extends CommandBase {
  Timer timer = new Timer();
  /** Creates a new IntakeSpinnerDefaultCommand. */
  public IntakeSpinnerDefaultCommand(Subsystem _sys) {
    addRequirements(_sys);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double sign = 1.0;
    double leftSpeed = RobotContainer.operatorController.getLeftTriggerAxis();
    double rightSpeed = RobotContainer.operatorController.getRightTriggerAxis();
    double speed = rightSpeed - leftSpeed;
    double curr = RobotContainer.intake.getIntakeCurrent();
    speed *= sign;
    if(Math.abs(speed) < .20){
      speed = 0;
      
    }else {
    switch(RobotContainer.arm.m_armController.m_armPos){
      
      case FAR_CUBE:
        speed = -1.0;
        break;
      case FLOOR_FRONT_CUBE: // Current to high
       
        if(curr > 30){
          speed = 0;
        }else {
          speed = 0.4;
        }
        break;
      case HOME:  // 0 or Wall speed with current limit
 
        if(curr > 30){
          speed = 0;
        }else {
          speed = .4;
        }
        break;
      case LOW_CONE:
        speed = -.4;
        break;
      case LOW_CUBE:
        speed = -.4;
        break;
      case MID_CONE:
      speed = -.4;
        break;
      case MID_CUBE:
        speed = -.4;
        break;
      case WALL_CONE:// Current limit
      if(curr > 30){
        speed = 0;
      }else {
        speed = .4;
      }
        break;
        case MID_CUBE_REVERSE:
        speed = -.6;
        break;
        case FAR_CUBE_REVERSE:
        speed = -1;
        break;
      default:
        break;

    }
  }


   // if(RobotContainer.operatorController.axisGreaterThan(2, 0.25).getAsBoolean() || RobotContainer.driverController.axisGreaterThan(2, 0.25).getAsBoolean()){
      RobotContainer.intake.spinHand(speed);
   // }else if(RobotContainer.operatorController.axisGreaterThan(3, 0.25).getAsBoolean() || RobotContainer.driverController.axisGreaterThan(3, 0.25).getAsBoolean()){
  //    RobotContainer.intake.spinHand(speed);
 //   }else {
 //     RobotContainer.intake.spinHand(0);
 //   }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
