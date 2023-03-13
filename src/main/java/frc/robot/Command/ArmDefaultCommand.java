// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystem.Arm;


public class ArmDefaultCommand extends CommandBase {
  Arm arm;
  double intakeCurrentAvg = 0;
  double intakeCurrentLim = 0;
  double intakeTimeLimit = 1;
  boolean in = false;
  boolean inLatch = false;
  int intakeCurrentCnt = 0;
  boolean out = false;
  double intakeSpeed = 1.0;
  Timer inTimer = new Timer();
  double inSpinCnts = 0;
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
