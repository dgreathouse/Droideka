// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command;


import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Subsystem.ArmSubsystem;

/**
 * Default Arm command controls the two motors by using a PID command.
 * Buttons:
 * 1. In robot and straiht up
 * 2. Wall Cone
 * 3. Wall Blob
 * 4. Floor front Blob
 * 5. Level Far Cone
 * 6. Level Mid Cone
 * 7. Level Low Cone
 * 8. Level Far Blob
 * 9. Level Mid Blob
 * 10. Level Low Blob
 * 
 * How will the cone be dropped on the post?
 * Option 1: Have a button move the postion ~10 lower than it is currently and release the cone
 * Option 2: Disable the PID and the weight of the arm will lower it while releasing the cone.
 * Option 3: ?
 * 
 * Currently this is only controlling one bicep motor. The other bicep motor is set to follow the Left motor.
 * The Forearm motor is not yet done in the ArmSubsystem. How the arm bicep and forearm motors are controlled together is to be determined
 */
public class ArmDefaultCommand extends CommandBase {
  SparkMaxPIDController left_PID;
  /** Creates a new ArmDefaultCommand. */
  public ArmDefaultCommand(ArmSubsystem _subsystem) {
    
    addRequirements(_subsystem);
  //  left_PID =  _subsystem.m_leftCanSparkMax.getPIDController();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
