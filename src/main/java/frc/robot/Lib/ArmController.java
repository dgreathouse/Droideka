// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Lib;


import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

    ArmDLGFeedForward m_shoulderFF;
    ArmDLGFeedForward m_elbowFF;
    ArmDLGFeedForward m_handFF;

    double m_intakeSpeed = 0;

    
    public ArmController(Arm _arm){
        // 45 Deg/sec 1.5 Rad/Sec^2
        m_shoulderPID = new ProfiledPIDController(10, 10, 0, new TrapezoidProfile.Constraints(2.5, 4));
        m_elbowPID = new ProfiledPIDController(20, 20, 0, new TrapezoidProfile.Constraints(35, 4));
        m_handPID = new ProfiledPIDController(15, 4, 0, new TrapezoidProfile.Constraints(4,3));

       // m_shoulderFF = new ArmDLGFeedForward(0.125, 0.55, 0.1);
    //    m_shoulderFF = new ArmDLGFeedForward(0.0, 0.0, 0.0);
    //     m_elbowFF = new ArmDLGFeedForward(0,0, 0);
    //     m_handFF = new ArmDLGFeedForward(0.0, 0.0, 0.0);
        m_handPID.setTolerance(0.01);
        m_elbowPID.setTolerance(.01);
        m_elbowPID.setIntegratorRange(-20, 20);
        arm = _arm;

        SmartDashboard.putNumber("ArmVolts", 0);
    }
    public void moveToPosition(){

        // Shoulder
        double angle = Math.toRadians(RobotContainer.armData.getBicepAngle(m_armPos));
        m_shoulderPID.setGoal(angle);
        double shPID = m_shoulderPID.calculate(Math.toRadians(arm.getShoulderAngle()));
       // double shVel = m_shoulderPID.getGoal().velocity;
       // double shFF = m_shoulderFF.calculate(Math.toRadians(arm.getShoulderAngle()-90), shVel);
        arm.moveShoulder(shPID);
        //arm.moveShoulder(SmartDashboard.getNumber("ArmVolts", 0));

        // Elbow
        double fAngle = Math.toRadians(RobotContainer.armData.getForearmAngle(m_armPos));
        m_elbowPID.setGoal(fAngle);
        double elPID = m_elbowPID.calculate(Math.toRadians(arm.getElbowAngle()));
       // double elVel = m_elbowPID.getGoal().velocity;
      //  double elFF = m_elbowFF.calculate(Math.toRadians(arm.getElbowAngle()-90), elVel);
        SmartDashboard.putNumber("elPID", elPID);
        //SmartDashboard.putNumber("elFF", elFF);
        arm.moveElbow(-elPID);
        //arm.moveElbow(SmartDashboard.getNumber("ArmVolts", 0));

        
        // Hand
        double hAngle = Math.toRadians((RobotContainer.armData.getHandAngle(m_armPos)));
        m_handPID.setGoal(hAngle);
        double hPID = m_handPID.calculate(Math.toRadians(arm.getHandAngle()));
      //  double hVel = m_handPID.getGoal().velocity;
     //   double hFF = m_handFF.calculate(Math.toRadians((arm.getHandAngle()+90)), hVel);
        arm.moveHand(hPID);
        //arm.moveHand(SmartDashboard.getNumber("ArmVolts", 0));

    }
}
