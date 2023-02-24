// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Lib;


import javax.swing.text.Position;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Subsystem.Arm;


/** Arm Controller
 * kV controls the feedforward gain for velocity
 * Or
 * PID controls the Velocity with max output being the set velocity
 * The PID uses the profile to come up with a velocity
 */
public class ArmController {
    
    Arm arm;
    ProfiledPIDController m_shoulderPID;
    ProfiledPIDController m_elbowPID;
    ProfiledPIDController m_handPID;

    ArmFeedforward m_shoulderFF;
    ArmFeedforward m_elbowFF;
    ArmFeedforward m_handFF;

    public ArmController(Arm _arm){
        m_shoulderPID = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
        m_elbowPID = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
        m_handPID = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));

        m_shoulderFF = new ArmFeedforward(0.125, 0.33, 0.1);
        m_elbowFF = new ArmFeedforward(0.1, 0.2, 0.1);
        m_handFF = new ArmFeedforward(0, 0, 0);

        arm = _arm;
    }
    public void moveToPosition(ArmPosEnum pos){

        m_shoulderPID.calculate(arm.getShoulderAngle());
        m_elbowPID.calculate(arm.getElbowAngle());
        m_handPID.calculate(0);


        /**
         * Calculate the PID value
         * Use the expected velocity as a FF velocity
         * Command each motor
         * Adjust shoulder kG based on elbow and intake angle, then intake angle for elbow
         */
    }

    



  

}
