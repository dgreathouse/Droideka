// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Lib;


import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.RobotContainer;
import frc.robot.Subsystem.Arm;


/** Arm Controller
 * kV controls the feedforward gain for velocity
 * Or
 * PID controls the Velocity with max output being the set velocity
 * The PID uses the profile to come up with a velocity
 */
public class ArmController {
    
    Arm arm;
    public ArmPosEnum m_armPos = ArmPosEnum.AUTO_HOME;

    ProfiledPIDController m_shoulderPID;

    ProfiledPIDController m_handPID;

    ArmDLGFeedForward m_shoulderFF;

    ArmDLGFeedForward m_handFF;

    double m_intakeSpeed = 0;

    
    public ArmController(Arm _arm){
        // All PIDs are in Degrees
        // Velocity is in 
        m_shoulderPID = new ProfiledPIDController(0.42, 0.05, 0, new TrapezoidProfile.Constraints(100,  160));
        m_handPID = new ProfiledPIDController(0.25, 0.5, 0, new TrapezoidProfile.Constraints(200,240));

        m_shoulderPID.setTolerance(0.01);
        m_handPID.setTolerance(0.01);

        arm = _arm;
    }
    public void moveToPosition(){

        // Shoulder
        double shoulderAngle = RobotContainer.armData.getBicepAngle(m_armPos);
        m_shoulderPID.setGoal(shoulderAngle);
        double shPID = m_shoulderPID.calculate(arm.getShoulderAngle());
        arm.moveShoulder(shPID);
        
        //Hand
        double handAngle = RobotContainer.armData.getHandAngle(m_armPos);
        m_handPID.setGoal(handAngle);
        double hPID = m_handPID.calculate(arm.getHandAngle());
        arm.moveHand(hPID);

    }
}
