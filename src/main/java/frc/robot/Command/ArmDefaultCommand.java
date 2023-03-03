// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Subsystem.Arm;


public class ArmDefaultCommand extends CommandBase {

  double bicepPos = 0;
  double elbowPos = 0;
  double intakePos = 0;
  double intakeVel = 0;

  /** Creates a new ArmDefaultCommand. */
  public ArmDefaultCommand(Arm _subsystem) {
    addRequirements(_subsystem);
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
      RobotContainer.arm.m_armController.moveToPosition();

      // Spin the intake based on the L/R trigger
      double lAxis = RobotContainer.operatorController.getLeftTriggerAxis();
      double rAxis = RobotContainer.operatorController.getRightTriggerAxis();
      double speed = lAxis - rAxis;
      RobotContainer.arm.spinHand(speed);
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
