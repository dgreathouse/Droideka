// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Lib.ArmController;
import frc.robot.Lib.ArmPosEnum;
import frc.robot.Subsystem.Arm;
/**
 * This will be the default command.
 * The Arm will just go to the position it is told to.
 * The buttons update a state and the PID should just go to that state
 * 
 * CONTROL
 * Feedforward with kS, kG, kV Static, Gravity, Velocity
 * PID Control to maintain and insure position (Low P, Hi I)
 * SetVoltage to compensate for battery sag.
 * 
 * ArmFeedForward adjusts the voltage based on the Angle. 
 * The Angle is different for the elbow based on the angle of the Shoulder
 * 
 */
public class ArmDefaultPIDCommand extends CommandBase {
  Arm m_arm;
  ArmController m_shouldController = new ArmController(0,0,0);
  ArmController m_elbowController = new ArmController(0,0,0);
  ArmController m_intakController = new ArmController(0,0,0);





  ArmPosEnum m_state = ArmPosEnum.HOME;

  /** Creates a new ArmDefaultPIDCommand. */
  public ArmDefaultPIDCommand(Arm _subsystem) {
    addRequirements(_subsystem);
    m_arm = _subsystem;

  }

  public void setState(ArmPosEnum _state){
    m_state = _state;

  }

 /**
  * 
  */
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

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
