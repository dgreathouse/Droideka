// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Lib;


import javax.swing.plaf.synth.SynthPasswordFieldUI;
import javax.swing.text.Position;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    public ArmPosEnum m_armPos = ArmPosEnum.HOME;

    ProfiledPIDController m_shoulderPID;
    ProfiledPIDController m_elbowPID;
    ProfiledPIDController m_handPID;

    ArmFeedforward m_shoulderFF;
    ArmFeedforward m_elbowFF;
    ArmFeedforward m_handFF;

    
    public ArmController(Arm _arm){
        // 45 Deg/sec 1.5 Rad/Sec^2
        m_shoulderPID = new ProfiledPIDController(6, 1, 0, new TrapezoidProfile.Constraints(.7854, 1.5));
        m_elbowPID = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
        m_handPID = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));

        m_shoulderFF = new ArmFeedforward(0.125, 0.33, 0.1);
        m_elbowFF = new ArmFeedforward(0.1, 0.2, 0.1);
        m_handFF = new ArmFeedforward(0, 0, 0);
        SmartDashboard.putData(m_shoulderPID);
        SmartDashboard.putData(m_elbowPID);
        SmartDashboard.putData(m_handPID);
        
        arm = _arm;
    }
    public void moveToPosition(){

        double angle = Math.toRadians(RobotContainer.armData.getBicepAngle(m_armPos));
        m_shoulderPID.setGoal(angle);
        double shPID = m_shoulderPID.calculate(Math.toRadians(arm.getShoulderAngle()));
      //  m_elbowPID.calculate(arm.getElbowAngle());
     //   m_handPID.calculate(0);

        double shVel = m_shoulderPID.getGoal().velocity;
        double shFF = m_shoulderFF.calculate(arm.getShoulderAngle(), shVel);

        arm.moveShoulder(shPID + shFF);


        /**
         * Calculate the PID value
         * Use the expected velocity as a FF velocity
         * Command each motor
         * Adjust shoulder kG based on elbow and intake angle, then intake angle for elbow
         */
    }

    



  

}
