// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Lib.ArmPosEnum;
import frc.robot.Subsystem.Arm;
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
      RobotContainer.armSubsystem.m_armController.moveToPosition(ArmPosEnum.FAR_CONE);
    
    // // Set the values for where to move based on the selected arm position.
    // switch(RobotContainer.armSubsystem.m_armPos){
    //   case FAR_CONE:
    //   RobotContainer.armSubsystem.setShoulderAngle(0);
    //   RobotContainer.armSubsystem.setElbowAngle(0);
    //   RobotContainer.armSubsystem.setIntakeAngle(0);
    //     break;
    //   case FAR_CUBE:
    //   RobotContainer.armSubsystem.setShoulderAngle(0);
    //   RobotContainer.armSubsystem.setElbowAngle(0);
    //   RobotContainer.armSubsystem.setIntakeAngle(0);
    //     break;
    //   case FLOOR_FRONT_CUBE:
    //   RobotContainer.armSubsystem.setShoulderAngle(0);
    //   RobotContainer.armSubsystem.setElbowAngle(0);
    //   RobotContainer.armSubsystem.setIntakeAngle(0);
    //     break;
    //   case HOME:
    //   RobotContainer.armSubsystem.setShoulderAngle(0);
    //   RobotContainer.armSubsystem.setElbowAngle(0);
    //   RobotContainer.armSubsystem.setIntakeAngle(0);
    //     break;
    //   case LOW_CONE:
    //   RobotContainer.armSubsystem.setShoulderAngle(0);
    //   RobotContainer.armSubsystem.setElbowAngle(0);
    //   RobotContainer.armSubsystem.setIntakeAngle(0);
    //     break;
    //   case LOW_CUBE:
    //   RobotContainer.armSubsystem.setShoulderAngle(0);
    //   RobotContainer.armSubsystem.setElbowAngle(0);
    //   RobotContainer.armSubsystem.setIntakeAngle(0);
    //     break;
    //   case MID_CONE:
    //   RobotContainer.armSubsystem.setShoulderAngle(0);
    //   RobotContainer.armSubsystem.setElbowAngle(0);
    //   RobotContainer.armSubsystem.setIntakeAngle(0);
    //     break;
    //   case MID_CUBE:
    //   RobotContainer.armSubsystem.setShoulderAngle(0);
    //   RobotContainer.armSubsystem.setElbowAngle(0);
    //   RobotContainer.armSubsystem.setIntakeAngle(0);
    //     break;
    //   case WALL_CONE:
    //   RobotContainer.armSubsystem.setShoulderAngle(0);
    //   RobotContainer.armSubsystem.setElbowAngle(0);
    //   RobotContainer.armSubsystem.setIntakeAngle(0);
    //     break;
    //   case WALL_CUBE:
    //   RobotContainer.armSubsystem.setShoulderAngle(0);
    //   RobotContainer.armSubsystem.setElbowAngle(0);
    //   RobotContainer.armSubsystem.setIntakeAngle(0);
    //     break;
    //   default:
    //   RobotContainer.armSubsystem.setShoulderAngle(0);
    //   RobotContainer.armSubsystem.setElbowAngle(0);
    //   RobotContainer.armSubsystem.setIntakeAngle(0);
    //     break;
    //   }
    //   // Call the arm methods to make the arm motors move
    //   RobotContainer.armSubsystem.rotateShoulder(RobotContainer.armSubsystem.m_shoulderAngle);
    //   RobotContainer.armSubsystem.rotateElbow(RobotContainer.armSubsystem.m_elbowAngle);
    //   RobotContainer.armSubsystem.rotateIntake(RobotContainer.armSubsystem.m_intakeAngle);
    //   // Set the intake velocity based on teleop or auto
    //   if(DriverStation.isTeleop()){
    //     intakeVel = RobotContainer.operatorController.getLeftTriggerAxis() - RobotContainer.operatorController.getRightTriggerAxis();
    //     RobotContainer.armSubsystem.spinIntake(intakeVel);
    //   }else if(DriverStation.isAutonomous()){
    //     intakeVel = RobotContainer.armSubsystem.m_intakeVelocity;
    //     if(RobotContainer.armSubsystem.onTarget()){
    //       RobotContainer.armSubsystem.spinIntake(intakeVel);
    //     }
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
